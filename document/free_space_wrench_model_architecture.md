# 현재 Free-space wrench 모델 architecture

## 동작 순서 요약

1. `ObserverInput`의 `q`, `dq`와 hardware-zeroed AFT wrench를 같은 시각으로 맞춘다.
2. 과거 값만 사용해 `qdd`를 만들고 최근 32 sample의 causal window를 유지한다.
3. follower URDF와 식별된 payload mass/CoM으로 자세별 중력 wrench를 계산한다.
4. 32-sample window를 54차원 multiscale feature로 바꾸고 ridge 모델로 남은 wrench를 예측한다.
5. `중력 wrench + ridge residual`을 최종 free-space wrench로 사용한다.
6. 측정 wrench에서 free-space 예측값을 빼 contact wrench를 계산한다.
7. force residual에 `2.5/1.2 N`, `12/20 ms` Schmitt 판정을 적용한다.
8. prediction/residual과 canonical `ContactObservation`을 발행한다.

## 현재 선택된 artifact

| 항목 | 값 |
|---|---|
| 모델 | `right_train13_ridge_short_multiscale_bundle_v3_20260822/model.ts` |
| 모델 SHA-256 | `8c61261bb2fdd0151291f9c52ca627e59e04d71bc5655c92df5081943280ee8b` |
| metadata SHA-256 | `025d761ba285d34850dfe4da1ba9b89d6f7c2109f9a03181fdfbadb55463d882` |
| 운용 선택 계약 | `operator_selected_20260901` |
| sample rate | `262.5 Hz` |
| 구조 | physical payload gravity + learned ridge residual |
| history | 32 sample, 약 122 ms |
| 출력 | sensor-frame `[Fx,Fy,Fz,Mx,My,Mz]` |

이 artifact는 기존 정량 정확도 gate를 통과했다는 의미가 아니라, 확인된 한계를
사용자가 수용하고 현재 운용 모델로 선택한 것이다. 다른 `approved=false` artifact는
허용하지 않으며, 향후 개선 모델은 정식 승인 계약을 통과하거나 새로운 명시적 선택
결정으로 이 두 SHA를 교체해야 한다.

## 전체 구조

```text
ObserverInput(q,dq,current_pose)       /aft_sensor2/wrench
               │                              │
               ├──── timestamp/frame 검증 ───┤
               │                              │
               ▼                              │
     causal [q,dq,qdd] 32-sample window       │
               │                              │
        ┌──────┴─────────┐                    │
        ▼                ▼                    │
 Pinocchio payload   54D multiscale           │
 gravity model       feature projection       │
        │                │                    │
        │                ▼                    │
        │          normalized ridge           │
        │          residual model             │
        └──────┬─────────┘                    │
               ▼                              │
       predicted free-space wrench            │
               │                              │
               └────────── measured - predicted
                                      │
                                      ▼
                         contact wrench / force norm
                                      │
                         Schmitt + time hold detector
                                      │
                 ┌────────────────────┼───────────────────┐
                 ▼                    ▼                   ▼
       ContactObservation     predicted_wrench     contact_wrench
       in right_base_link     in aft_sensor2       in aft_sensor2
```

## 1. 입력과 시간 계약

모델의 base feature는 다음 18개 값이다.

```text
[q_rad[6], dq_rad_s[6], causal_qdd_rad_s2[6]]
```

- robot state: `/contact_state/observer_input`
- physical FT: `/aft_sensor2/wrench`
- observer/model rate: `262.5 Hz`
- robot state frame: `right_base_link`
- FT/model frame: `aft_sensor2`
- 최대 source age: `20 ms`
- 최대 state/FT timestamp 차이: `3 ms`

`qdd`는 현재 `dq`와 바로 이전 `dq`의 차분만 사용한다. 미래 sample, measured
wrench, joint torque, task error는 feature에 포함하지 않는다. 접촉 신호가 입력으로
누설되어 모델이 실제 접촉까지 free-space로 지우는 것을 막기 위해서다.

## 2. 고정 자세 zero 검증

observer는 launch 인자만 믿고 바로 예측하지 않는다. 다음 조건을 1초 동안 다시
확인한 뒤 history를 채운다.

- zero pose: `[5.5, 52, 112, 28, -107, -35] deg`
- joint pose 오차: `1 deg 이하`
- joint speed: `0.02 rad/s 이하`
- 1초 force median vector norm: `1 N 이하`
- force 축별 STD: `0.40 N 이하`

검증 전과 history warm-up 중에는 `valid=false`다.

## 3. Payload gravity 물리 모델

학습 train group 중 acceleration이 하위 25%인 sample로 payload를 식별했다.

```text
mass = 1.6337626209 kg
CoM_sensor = [-0.0027390, -0.0065847, 0.1123464] m
```

Pinocchio가 URDF의 현재 자세에서 sensor-frame gravity 방향을 계산한다. Hardware
zero pose에서 이미 제거된 중력 성분을 다시 더하지 않도록 zero pose의 gravity를
뺀 변화량만 사용한다.

```text
delta_g(q) = g_sensor(q) - g_sensor(q_zero)
F_gravity  = mass * delta_g(q)
M_gravity  = CoM_sensor × F_gravity
W_gravity  = [F_gravity, M_gravity]
```

URDF 파일은 metadata에 기록된 SHA-256과 다르면 로딩을 거부한다.

## 4. 32-sample multiscale feature

최근 32 sample에서 다음 54차원 feature를 만든다.

```text
sin(q_now)                  6
cos(q_now)                  6
dq_now                      6
mean(dq, last 8/16/32)     18
mean(qdd, last 8/16/32)    18
total                       54
```

짧은 구간과 긴 구간의 속도·가속도 평균을 함께 사용해 순간 운동과 약 122 ms의
운동 이력을 선형 모델 하나로 표현한다.

## 5. Ridge residual 모델

학습 target에서 물리 중력 예측을 먼저 뺀 값이 ridge target이다.

```text
W_residual_target = W_sensor - W_gravity
```

54차원 입력과 6축 residual target을 train 통계로 표준화한 뒤 L2 정규화 계수
`1.0`인 ridge regression을 학습했다. 계수와 normalization은 단일 TorchScript
linear model에 포함된다.

```text
x_norm = (x - x_mean) / x_std
y_norm = x_norm @ coefficient
W_ridge = y_norm * y_std + y_mean
```

## 6. 최종 free-space 예측

```text
W_free_hat = W_gravity + W_ridge
```

현재 bundle의 model-only benchmark는 p99 `0.08783 ms`, 최대 `0.60003 ms`로
262.5 Hz 주기 `3.80952 ms` 안에 들어온다.

## 7. Contact residual과 상태 판정

```text
W_contact_hat = W_sensor - W_free_hat
e_force       = norm(W_contact_hat[Fx,Fy,Fz])
```

- FREE → CONTACT: `e_force >= 2.5 N`가 `12 ms` 이상 지속
- CONTACT → FREE: `e_force <= 1.2 N`가 `20 ms` 이상 지속
- `1.2 N < e_force < 2.5 N`: 현재 상태 유지
- moment는 상태 판정에 사용하지 않음

입력이 invalid/stale/unsynchronized이거나 model/history가 준비되지 않으면 detector
state를 초기화하고 invalid observation을 발행한다. Leader feedback 소비자는 이때
반사 torque를 0으로 만들어야 한다.

## 8. 출력과 소비자

| Topic | Frame | 내용 |
|---|---|---|
| `/contact_observer/right/observation` | `right_base_link` | residual, prediction, state, validity, timing |
| `/ft_free_space/right/predicted_wrench` | `aft_sensor2` | sensor-frame free-space prediction |
| `/ft_free_space/right/contact_wrench` | `aft_sensor2` | sensor-frame measured minus prediction |
| `/ft_contact_observer/diagnostics` | JSON | acceptance source, zero/history/rate/failure 상태 |

Leader teleoperation과 IL recorder는 첫 번째 canonical observation만 소비한다. 별도
contact publisher나 leader 내부 중복 contact detector를 동시에 사용하지 않는다.

## 관련 구현

- Feature와 detector: `ft_fb_leaderarm/contract.py`
- Model bundle과 physical prediction: `ft_fb_leaderarm/model.py`
- Runtime observer: `ft_fb_leaderarm/observer_node.py`
- Bundle 생성: `scripts/finalize_free_space_ridge_bundle.py`
- 단계별 pseudocode: [free_space_wrench_model_pseudocode.md](free_space_wrench_model_pseudocode.md)
