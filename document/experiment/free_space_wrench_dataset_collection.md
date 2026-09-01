# Free-space wrench 학습 dataset 수집 실험

> 이 문서는 2026-08 데이터 수집과 당시 모델 선택 결과를 보존한 실험 기록이다.
> 현재 운용 모델과 계약은
> [현재 architecture](../free_space_wrench_model_architecture.md)를 따른다.

- 수행일: 2026-08-19
- 대상: 오른팔 follower, `/aft_sensor2`, feedback-OFF leader teleoperation
- 범위: FAST 상태의 완전 무접촉 task 3회와 다양한 궤적 10회

## 목적

로봇이 물체와 접촉하지 않고 움직일 때 physical AFT가 측정하는 6축 wrench를
robot state `[q,dq,causal-qdd]`와 함께 `262.5 Hz`로 저장한다. 모델은 이를 이용해
free-space wrench를 예측하며 observer의 contact wrench는 다음 계약을 사용한다.

```text
contact_wrench = bias-removed physical FT raw - predicted free-space wrench
```

task 궤적만 학습했을 때 다른 경로에 과적합되는 것을 줄이기 위해 task 3개와
위치·자세·속도 범위를 나눈 diverse 10개를 서로 독립적인 hardware zero-set으로
수집했다.

## 안전·데이터 계약

- 시작 자세: `[5.5, 52.0, 112.0, 28.0, -107.0, -35.0] deg`
- tool/payload/controller/AFT frame을 전 episode에서 바꾸지 않는다.
- cold start한 AFT는 첫 formal 수집 전 120분 warm-up을 한 번 수행한다.
- 각 episode 전에 기준 자세에서 hardware zero-set을 하고 고유 `zero_set_id`를 쓴다.
- `CURRENT/SLOW`는 정렬에만 사용하고 저장하지 않는다. `FAST`만 기록한다.
- 모든 동작은 완전 무접촉이며 cable이 당겨지지 않는 작업공간에서 수행한다.
- robot/leader 이동 키는 운용자가 주변 안전을 확인한 뒤 직접 누른다.

## 실행 명령

### PC 공통 환경

```bash
source /opt/ros/humble/setup.bash
source /home/vision/contact_pipeline_ws/install/setup.bash
source /home/vision/dualarm_ws/install/setup.bash
source /home/vision/venv_act/bin/activate

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=7
export ROS_LOCALHOST_ONLY=0
export CYCLONEDDS_URI=file:///home/vision/cyclonedds_dualarmPC.xml
export ROS_LOG_DIR=/tmp/ft_fb_leaderarm_ros_logs

export FT_DATA_DIR=/home/vision/.ros/ft_fb_leaderarm/datasets/right_diverse10_20260819
export FT_PAYLOAD_ID=right_tool_m2p1kg_oz0p17m_v1
export FT_CONTROLLER_HASH=bae_r_v2_c113eabf7e13_ca07ae197213
mkdir -p "${FT_DATA_DIR}"
```

PC-SBC 시계와 입력을 확인한다.

```bash
sudo -v
/home/vision/dualarm_ws/src/fb_leaderarm/scripts/dualarm_chrony_mode.sh status

ros2 topic info /contact_state/observer_input --verbose
ros2 topic info /aft_sensor2/wrench --verbose
timeout 15s ros2 topic hz /contact_state/observer_input
timeout 15s ros2 topic hz /aft_sensor2/wrench
```

### SBC 장비 실행과 zero-set

robot driver, 일반 V2 impedance controller, hand driver와 AFT driver 명령은
[실행 명령](../command.md)의 2~4절을 따른다. AFT를 새로 시작했을 때 500 Hz를
한 번 명시한다.

```bash
ros2 topic pub --once /aft_sensor2/sample_rate_setting \
  std_msgs/msg/Int32 "{data: 500}"
```

각 episode 전에 기준 자세·정지·무접촉을 확인한 뒤 zero-set한다.

```bash
ros2 topic info /aft_sensor2/bias_setting --verbose
ros2 launch aft_can_hardware aft_zero_set2.launch.py \
  sensor_name:=aft_sensor2
```

### Collector/GUI와 episode

각 zero-set마다 ID만 바꿔 통합 launch를 새로 실행한다.

```bash
ros2 launch ft_fb_leaderarm collect_free_space_gui.launch.py \
  output_dir:="${FT_DATA_DIR}" \
  zero_set_confirmed:=true \
  zero_set_id:=tare_YYYYMMDD_01 \
  payload_id:="${FT_PAYLOAD_ID}" \
  controller_config_hash:="${FT_CONTROLLER_HASH}" \
  start_teleop:=true
```

GUI가 `VERIFIED / IDLE / IDLE / OK`일 때 다음 순서로 수행한다.

```text
1 → ARMED 확인 → c → CURRENT 확인 → t → SLOW 정렬 완료
→ o → FAST/RECORDING 확인 → 무접촉 궤적
→ 완전 정지 → s → 2
```

종료는 GUI에서 `q` 한 번, `SHUTDOWN Done` 확인, launch terminal에서
`Ctrl+C` 한 번 순서다. `start_teleop:=true`는 startup에서 leader를 follower
자세로 실제 ALIGN할 수 있지만 follower command는 발행하지 않는다.

### 저장 파일과 dataset 검증

최근 JSON은 Python 표준 기능으로 확인할 수 있다.

```bash
LATEST_JSON="$(ls -1t "${FT_DATA_DIR}"/right_free_space_*.json | head -1)"
LATEST_NPZ="${LATEST_JSON%.json}.npz"

python3 -c 'import json,sys; m=json.load(open(sys.argv[1])); print(json.dumps({k:m.get(k) for k in ("accepted","zero_set_id","record_only_teleop_state","samples","duration_s","actual_hz","max_record_gap_ms","sync_rejections","invalid_rejections")},indent=2))' "${LATEST_JSON}"

unzip -t "${LATEST_NPZ}"
```

3개 이상의 독립 zero group을 모은 뒤 validator를 실행한다. output 파일이 이미
있으면 덮어쓰지 않으므로 새 이름을 사용한다.

```bash
source /home/vision/venv_act/bin/activate
ros2 run ft_fb_leaderarm ft_free_space_validate -- \
  --data-dir "${FT_DATA_DIR}" \
  --seed 7 \
  --output "${FT_DATA_DIR}/dataset_validation_v1.json"
```

## 수집 방법

### FAST-only task pilot 3개

동일한 모방학습 task를 서로 다른 hardware zero-set에서 3회 수행했다. 각 episode는
FAST 상태에서만 16~22초 기록했으며 접촉 전에 종료했다.

### Diverse 10개

한 episode에 모든 동작을 넣지 않고, 전체 10개가 합쳐서 상태 공간을 덮도록
역할을 나눴다.

| zero-set | 주 궤적 |
|---|---|
| `div01` | 좌우 왕복, J1 변화 중심 |
| `div02` | 가까이/멀리, J2/J3 변화 중심 |
| `div03` | 위/아래 왕복 |
| `div04` | 손목 J4/J5/J6 양방향 회전 |
| `div05` | Cartesian X 왕복 |
| `div06` | Cartesian Y 왕복 |
| `div07` | Cartesian Z 왕복 |
| `div08` | 대각선·곡선 3차원 이동 |
| `div09` | 위치 이동과 손목 회전 결합 |
| `div10` | 실제 task와 유사한 무접촉 변형 경로 |

각 episode는 시작/종료 정지 구간, 양방향 반복, 느림/보통/조금 빠른 속도와 완만한
가감속을 포함했다. `FAST`는 teleop 상태 이름이며 최대 속도 지시가 아니다.

## 결과

### Task pilot

| zero-set | 파일 | 시간 [s] | samples | 실제 Hz | 최대 gap [ms] | 결과 |
|---|---|---:|---:|---:|---:|---|
| `tare_20260819_02` | `right_free_space_20260819_034917` | 16.495 | 4,331 | 262.504 | 4.014 | PASS |
| `tare_20260819_03` | `right_free_space_20260819_040748` | 21.775 | 5,717 | 262.503 | 4.037 | PASS |
| `tare_20260819_04` | `right_free_space_20260819_041307` | 20.655 | 5,423 | 262.503 | 4.021 | PASS |

- 합계: 3 episodes, 3 groups, 15,471 samples, 58.925초
- 세 파일 모두 FAST-only, `accepted=true`, sync/invalid rejection 0이다.
- pilot 5개 ablation 중 `history_mlp`가 선택됐지만 validation/test 최대 force-vector
  오차가 `4.295/2.733 N`으로 `1 N` 기준을 실패했다.
- model-only 추론 성능은 p99 기준 약 `38,300 Hz`, 최악값 기준 약
  `20,500 Hz`(`0.0261/0.0489 ms`)로 통과했다.

### Diverse 10개

| zero-set | 파일 | 시간 [s] | samples | 실제 Hz | 최대 gap [ms] | sync reject | 결과 |
|---|---|---:|---:|---:|---:|---:|---|
| `div01` | `044348` | 58.674 | 15,403 | 262.501 | 4.034 | 0 | PASS |
| `div02` | `044927` | 62.148 | 16,315 | 262.502 | 4.029 | 6 | PASS |
| `div03` | `045441` | 64.746 | 16,997 | 262.503 | 4.027 | 0 | PASS |
| `div04` | `050000` | 82.788 | 21,733 | 262.502 | 4.025 | 0 | PASS |
| `div05` | `050511` | 78.730 | 20,668 | 262.505 | 4.027 | 3 | PASS |
| `div06` | `050941` | 81.248 | 21,329 | 262.505 | 4.037 | 0 | PASS |
| `div07` | `051410` | 80.342 | 21,091 | 262.503 | 6.000 | 0 | PASS |
| `div08` | `051928` | 117.274 | 30,786 | 262.505 | 5.001 | 1 | PASS |
| `div09` | `052432` | 109.256 | 28,681 | 262.503 | 6.001 | 0 | PASS |
| `div10` | `053045` | 153.582 | 40,317 | 262.505 | 6.006 | 1 | PASS |

- 합계: 10 episodes, 10 independent zero groups, 233,320 samples,
  888.788초(14분 48.8초)
- validator split: train/validation/test `6/2/2 groups`
- payload, controller hash, FT frame, observer frame와 zero pose가 모두 일치했다.
- sync reject 11개는 저장 전에 제외됐다. 보존 sample의 최대 sync error는 모두
  3 ms 이내였고 record gap은 모두 10 ms 이하였다.
- 공식 validator 결과는 PASS다.

## 제외 데이터와 문제 연결

- `right_free_space_20260819_034238.npz`: 이전/새 collector가 동시에 같은 파일을
  기록해 내부 metadata와 외부 JSON이 다르고 두 array 압축이 손상됐다. 원본은
  삭제하지 않고 pilot에서 제외했다. 상세 기록은
  [FT-20260808-01의 FAST pilot 절](../problem/FT-20260808-01.md#2026-08-19-fast-only-task-pilot와-5개-ablation)에 있다.
- `right_free_space_20260819_025951.npz`: SLOW smoke이며 CRC 오류가 있어 제외했다.
- 첫 SLOW episode의 74.757초 길이 초과:
  [FT-20260811-04](../problem/FT-20260811-04.md)
- legacy zero-set 종료 교착:
  [FT-20260808-02](../problem/FT-20260808-02.md)
- zero 기준 자세 부호 불일치:
  [FT-20260808-03](../problem/FT-20260808-03.md)
- AFT 설정 1000 Hz와 실제 500 Hz 갱신 불일치:
  [FT-20260808-04](../problem/FT-20260808-04.md)
- launch Ctrl-C cleanup 중복 SIGINT:
  [FT-20260808-05](../problem/FT-20260808-05.md)
- Chrony helper 상대경로 오류:
  [FT-20260811-01](../problem/FT-20260811-01.md)
- `feedback_source:=off` YAML type 오류:
  [FT-20260811-02](../problem/FT-20260811-02.md)
- startup leader 자동 ALIGN 안내 누락:
  [FT-20260811-03](../problem/FT-20260811-03.md)

## Artifact와 현재 상태

- task pilot:
  `/home/vision/.ros/ft_fb_leaderarm/datasets/right_task3_pilot_20260819/`
- diverse 10:
  `/home/vision/.ros/ft_fb_leaderarm/datasets/right_diverse10_20260819/`
- diverse validator:
  `/home/vision/.ros/ft_fb_leaderarm/datasets/right_diverse10_20260819/dataset_validation_v1.json`
- rejected pilot model/report:
  `/home/vision/.ros/ft_fb_leaderarm/models/right_task3_pilot_20260819/`

## Final 13-group 학습 결과

3개 task episode와 10개 diverse episode를 새 clean dataset으로 복사했다. 원본은
변경하지 않았다.

- final dataset:
  `/home/vision/.ros/ft_fb_leaderarm/datasets/right_final13_20260819/`
- dataset validator: 13 episodes, 13 groups, 248,791 samples, 947.713초, PASS
- split: train/validation/test `7/3/3` independent zero groups
- model/report:
  `/home/vision/.ros/ft_fb_leaderarm/models/right_final13_20260819/`

| 후보 | validation force max [N] | p95 [N] | RMSE [N] |
|---|---:|---:|---:|
| static linear | 5.793 | 2.565 | 1.257 |
| dynamic MLP | **3.980** | 1.687 | 0.951 |
| history MLP | 4.409 | 1.696 | 0.912 |
| history LSTM | 4.871 | **1.624** | **0.882** |
| history GRU | 5.054 | 1.661 | 0.895 |

validation 최대 오차가 가장 낮은 `dynamic_mlp`를 선택하고 이 모델만 held-out test에
사용했다. test force max/p95/RMSE는 `6.260/1.460/0.857 N`으로 정확도 gate를
실패했다. CPU model-only 추론 성능은 p99 기준 약 `41,900 Hz`, 최악값 기준 약
`32,400 Hz`(`0.02388/0.03088 ms`)로 요구값 `262.5 Hz`를 통과했다.
`metadata.approved=false`이며 모델을 observer/runtime에 사용하지 않는다.

상세 분석과 artifact hash는
[FT-20260819-01](../problem/FT-20260819-01.md)에 기록했다.

## 평가지표 의미와 해석 방법

각 sample에서 측정 force를 `F`, 예측 force를 `F_hat`이라고 하면 force-vector
오차는 다음과 같다.

```text
e_force = ||F - F_hat||_2
        = sqrt((Fx-Fx_hat)^2 + (Fy-Fy_hat)^2 + (Fz-Fz_hat)^2)
```

| report 항목 | 의미 | 해석 |
|---|---|---|
| `force_norm_max_n` | 모든 sample 중 `e_force`의 최댓값 | 한 번이라도 발생한 최악 오차다. 본 실험의 최종 합격 기준은 validation과 새 held-out test 모두 `<= 1 N`이다. |
| `force_norm_p95_n` | `e_force`의 95 percentile | sample의 95%가 이 값 이하이고 5%는 이 값보다 크다. max만 큰지, 큰 오차가 자주 발생하는지 구분한다. |
| `force_norm_rmse_n` | `sqrt(mean(e_force^2))` | 전체적인 오차 크기이며 큰 오차에 더 큰 가중치가 생긴다. `1 N` 미만이어도 max가 `1 N`을 넘으면 합격이 아니다. |
| `force_axis_abs_max_n` | Fx/Fy/Fz 각 축의 최대 절대 오차 | 어떤 방향이 최악 오차를 만드는지 진단한다. 3개 축을 합친 최종 gate는 `force_norm_max_n`이다. |
| `moment_axis_abs_max_nm` | Mx/My/Mz 각 축의 최대 절대 오차 | torque 방향별 최악 오차다. 현재 `1 N` force gate와는 별도 진단값이다. |
| `wrench_axis_rmse` | Fx/Fy/Fz/Mx/My/Mz 각 축 RMSE | 축별 평균 성능을 비교한다. 앞 3개 단위는 N, 뒤 3개 단위는 Nm다. |
| validation | 모델·checkpoint 선택용 독립 zero group | 후보 선택과 개선 판단에 사용할 수 있다. |
| held-out test | 선택이 끝난 모델을 한 번만 확인하는 독립 zero group | 한 번 열어 본 뒤에는 다음 모델 선택에 재사용하지 않는다. 변경 모델에는 새 독립 test가 필요하다. |

지표는 `max → p95 → RMSE → 축별 값` 순서로 읽는다. 예를 들어 final13 test의
`6.260/1.460/0.857 N`은 평균 수준인 RMSE만 보면 1 N 아래지만, 약 5%의 sample이
1.460 N보다 크고 최악 순간은 6.260 N이므로 contact observer용으로는 실패라는
뜻이다.

추론 속도는 앞으로 Hz를 먼저 기록한다. 단일 model call latency가 `L ms`이면
model-only 등가 처리율은 `1000/L Hz`다.

| 측정값 | final13 결과 | 해석 |
|---|---:|---|
| 평균 latency 등가 처리율 | 약 `47,100 Hz` | 평균 latency `0.02122 ms`의 역수 |
| p99 latency 등가 처리율 | 약 `41,900 Hz` | call의 99%가 `0.02388 ms` 이내였고 그 latency의 역수 |
| 최악 latency 등가 처리율 | 약 `32,400 Hz` | 관측한 가장 느린 model call 기준 |
| 요구 publish rate | `262.5 Hz` | 한 sample당 약 `3.810 ms` |

위 Hz는 전처리와 ROS publish를 제외한 model-only 등가 처리율이다. 실제 observer의
유효 publish rate `>= 262.5 Hz`는 승인 모델이 생긴 뒤 FS-05에서 따로 측정한다.

## Task와 다양한 궤적을 분리한 결과

여기서 `diverse 10`은 난수 생성기로 만든 진짜 random trajectory가 아니라, 운용자가
위치·자세·속도 역할을 나눠 수행한 다양한 궤적 10개다.

### Task-only pilot 모델의 독립 group 결과

task 3개만 사용해 한 group씩 train/validation/test로 나눈 앞선 `history_mlp` 결과다.

| 범위 | samples | force max [N] | p95 [N] | RMSE [N] | 결과 |
|---|---:|---:|---:|---:|---|
| task validation 1 group | 4,316 | 4.295 | 1.864 | 1.083 | FAIL |
| task held-out test 1 group | 5,408 | 2.733 | 1.923 | 1.213 | FAIL |

동일 task를 반복해도 zero-set이 달라지면 `1 N`을 보장하지 못했다. 다만 group이
각 split에 하나뿐이어서 성능 분산을 안정적으로 추정하기에는 적다.

### Final13 `dynamic_mlp`를 궤적 종류별로 다시 평가한 결과

| 범위 | split 성격 | samples | force max [N] | p95 [N] | RMSE [N] |
|---|---|---:|---:|---:|---:|
| task 3개 | 모두 train에 포함된 in-sample | 15,471 | 3.903 | 1.303 | 0.743 |
| diverse train 4개 | in-sample | 88,740 | 2.874 | 1.203 | 0.695 |
| diverse validation 3개 | 모델 선택용 | 58,074 | 3.980 | 1.687 | 0.951 |
| diverse held-out test 3개 | 이미 한 번 평가함 | 86,506 | 6.260 | 1.460 | 0.857 |
| diverse 전체 10개 | train/validation/test 혼합 참고값 | 233,320 | 6.260 | 1.443 | 0.825 |

task 3개 값은 모델이 이미 학습한 sample의 결과이므로 diverse validation/test와 직접
비교해 일반화가 더 좋다고 결론 내릴 수 없다. 공정한 근거는 task-only pilot의 독립
validation/test이며, 그 결과도 모두 `1 N` gate를 실패했다. 다양한 궤적에서는
특히 새 동작에 해당하는 validation/test의 tail 오차가 커졌다.

## Final13 residual 원인 분석

선택 모델을 13개 전체 episode에 다시 적용해 `target - prediction`을 분석했다. 이는
사후 진단이며 이미 공개된 test를 다음 모델 선택에 사용하지 않는다.

### Offset·zero-set·시간 영향

- episode별 평균 force residual norm은 `0.050~0.616 N`이었다.
- 각 episode의 실제 평균 residual을 완벽히 알고 제거하는 비현실적인 oracle 보정도
  전체 max/p95/RMSE를 `6.260/1.436/0.820 N`에서
  `6.261/1.357/0.770 N`으로만 바꿨다.
- episode 평균 offset이 차지한 force squared error 비율은 약 `11.9%`였다.
- 시작 1초의 residual을 offset으로 사용한 진단 보정은
  `6.036/1.453/0.818 N`으로 p95를 개선하지 못했다.
- diverse episode의 처음 10%와 마지막 10% 평균 residual 차이는
  `0.108~0.349 N`이었다. 궤적 자체가 시간과 함께 바뀌므로 이것만으로 thermal
  drift라고 단정할 수 없다.

따라서 warm-up과 매 episode zero-set은 계속 필요하지만, 절차를 안정화하거나 상수
offset만 빼는 것으로 `1 N` 목표를 달성할 수 있다는 근거는 없다.

### 움직임·가속도 영향

전체 13개에서 joint speed norm `<= 0.05 rad/s`를 정지에 가까운 구간으로 나눴다.

| 구간 | samples | force max [N] | p95 [N] | RMSE [N] |
|---|---:|---:|---:|---:|
| 정지에 가까움 | 45,882 | 2.360 | 1.014 | 0.581 |
| 이동 중 | 202,909 | 6.260 | 1.492 | 0.865 |

속도 사분위가 낮은 구간부터 높은 구간까지 p95는
`1.093 → 1.359 → 1.470 → 1.667 N`, RMSE는
`0.631 → 0.801 → 0.867 → 0.950 N`으로 증가했다. 가속도 사분위도 p95가
`1.085 → 1.346 → 1.461 → 1.684 N`으로 증가했다. force error norm과
speed/acceleration norm의 Pearson 상관은 각각 `0.252/0.316`으로, 강한 단일
원인은 아니지만 움직임이 커질수록 오차가 증가하는 일관된 경향이 있다.

episode별로 force residual 각 축과 joint position 각 축의 절대 Pearson 상관 중
최댓값은 `0.093~0.512`였다. `div02`는 `0.512`, `div07`은 `0.361`이지만 나머지는
대체로 더 작아 자세 영향이 일부 보이되 모든 궤적에 공통된 단일 원인으로 확인되지는
않았다.

가장 큰 `6.260 N` 오차는 `div08`의 약 `100.389초` 지점에서 발생했으며 당시
speed/acceleration norm은 약 `0.715 rad/s`, `9.059 rad/s²`였다. test의 최대
가속도는 `15.664 rad/s²`로 train 최대 `8.399 rad/s²`를 넘었다. validation의
`div02/div05/div07`도 각각 sample의 약 `9.74/7.51/11.28%`가 train feature
범위를 벗어났다. 다만 `div01`은 범위를 벗어난 sample이 없어도 max `2.691 N`이므로
입력 범위 부족만이 유일한 원인은 아니다.

validation에서 예측과 측정을 `-16~+16 samples` 이동해 확인했을 때 RMSE 최적은
`+1 sample`이었지만 `0.9511 → 0.9499 N`의 미미한 변화였고 max는 오히려
`3.980 → 4.638 N`으로 증가했다. 단일한 global timestamp shift만 고쳐서 해결될
문제라는 근거도 없다.

### 객관적인 판단과 다음 순서

현재 evidence의 우선순위는 다음과 같다.

1. 주된 개선 대상은 상수 offset보다 이동·가속 구간의 동적 residual과 train 범위
   밖 상태다.
2. 정지 구간도 max `2.360 N`이므로 zero/노이즈 문제는 함께 남아 있다.
3. 현재처럼 서로 비슷한 tail 오차를 가진 모델을 단순 평균하는 ensemble은 공통
   residual을 제거하지 못하므로 첫 조치로 사용하지 않는다.
4. validation-only qdd smoothing과 짧은 state history 비교를 수행했으며 단일 모델
   최선 max는 `3.947 N`으로 실패했다.
5. 사용자 요청에 따라 같은 비율의 제한된 ensemble도 확인했지만 최선 max는
   `3.530 N`, residual 상관은 `0.726`으로 실패했다. weighted ensemble은 추가하지
   않는다.
6. targeted train data로 방법을 개선한 뒤 새 hardware zero-set의 task와 diverse
   episode를 최소 3개 별도 수집해 새 held-out test로 판정한다. 기존 test는 다시
   선택 기준으로 쓰지 않는다.

이 분석은 저장 파일만 사용했으며 로봇 이동, AFT zero-set 또는 sensor 실행은 하지
않았다.

이후 수행한 qdd smoothing, short history, 단순 ensemble과 data-size learning curve는
[feature·ensemble·data-size ablation](free_space_wrench_feature_ablation.md)에
기록했다.
