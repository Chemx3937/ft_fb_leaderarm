# Acceptance contract

## Purpose

목표 구현과 단계 승인의 안정적인 기준이다. 코드가 있어도 실측 evidence가 없으면
합격이 아니다. 현재 진행 상태는 `document/TODO_LIST.md`, 운용은
`document/flow.md`, gate별 검증 경로는 `docs/verification.md`를 따른다.

## Shared contract

- 오른팔 manipulator 시작 조인트 각도는
  `[5.5, 52.0, 112.0, 28.0, -107.0, -35.0] deg`다.
- 수집, 학습, observer는 `262.5 Hz` 계약을 공유한다. AFT 센서의 유효 취득률
  약 `500 Hz`는 상위 source rate이며 모델 호출·contact 판정 주기가 아니다.
- runtime bundle은 정식 `approval_contract=robust_force_v2_262p5hz`를 통과하거나,
  아래에 고정한 operator-selected model/metadata SHA 쌍과 정확히 일치해야 한다.
- dataset과 model의 `zero_set_id`, payload, controller, sensor frame 계약이
  runtime과 일치해야 한다.
- 로봇 이동, FT zero-set, 데이터 수집, teleoperation, feedback은 사용자 승인 없이
  실행하지 않는다.

## Wrench contract

초기 bias wrench를 `b0`, zero-set 전 센서 출력을 `W_raw`라고 하면 현재 계약은
다음과 같다.

```text
W_sensor(t)       = W_raw(t) - b0
W_contact_hat(t)  = W_sensor(t) - W_free_hat(t)
e_force(t)        = ||F_sensor(t) - F_free_hat(t)||_2
```

`/aft_sensor2/wrench`는 hardware zero-set으로 `b0`가 한 번 제거된 sensor-frame
wrench다. Free-space model은 무접촉 `W_sensor`를 예측하고, observer는 현재처럼
예측값만 빼야 한다. 별도 software bias 제거를 추가하려면 수집·학습·runtime·테스트의
계약을 같은 변경에서 수정한다.

## Operator-selected runtime model

2026-09-01 사용자는 현재 모델의 알려진 정확도 한계를 수용하고 아래 artifact를
운용 모델로 선택했다. 이 결정은 `FS-03`을 PASS로 바꾸지 않으며, observer-only와
feedback 단계의 실측 gate도 면제하지 않는다.

```text
acceptance source : operator_selected_20260901
model SHA-256     : 8c61261bb2fdd0151291f9c52ca627e59e04d71bc5655c92df5081943280ee8b
metadata SHA-256  : 025d761ba285d34850dfe4da1ba9b89d6f7c2109f9a03181fdfbadb55463d882
```

두 SHA 중 하나라도 다르면 runtime, feedback evidence 분석과 authorization이 모두
거부해야 한다. 향후 개선 모델이 정식 `robust_force_v2_262p5hz` 계약을 통과하면
별도 operator-selected 예외 없이 교체할 수 있다.

## Free-space force prediction gates

| ID | 합격 기준 | Evidence |
|---|---|---|
| `FS-01` | 매 수집 session 시작 전 기준 자세 오차 `<= 1 deg`, 정지 및 hardware zero 확인 | collector metadata와 zero diagnostics |
| `FS-02` | contact-free dataset, 독립 `zero_set_id` 3개 이상, payload/controller/frame 혼합 없음 | dataset validation report |
| `FS-03` | validation과 선택 모델의 새 held-out test 각각 aggregate `p99(e_force) <= 1 N`, 모든 `zero_set_id` group `p95(e_force) <= 1 N`, hard max `<= 2 N` | `ablation_report.json` |
| `FS-04` | target PC model-only inference `p99 <= 3.048 ms`, hard max `<= 3.810 ms` | model runtime benchmark |
| `FS-05` | observer ready 이후 `valid_predictions >= floor(duration_s * 262.5)`, deadline miss·invalid·stale 0회 | 독립 runtime report |
| `FS-06` | 독립 무접촉 동작에서 residual force p95 `<= 1.2 N`, p99 `<= 1.5 N`, hard max `<= 2.5 N`, false CONTACT 0회 | observer-only FREE report |
| `FS-07` | 전용 GUI에서 leader 제어, 안전 순서 강제, 수집, dataset 검증과 학습 실행·상태 확인 가능 | GUI integration test |

모델 선택에는 validation만 사용한다. `FS-03` validation을 통과한 후보 중 RMSE가 가장
작은 모델을 선택하고 held-out test는 그 모델에 한 번만 사용한다. 통과 후보가 없으면
최저 RMSE 후보는 기본적으로 diagnostic artifact로만 남긴다. 위의 명시적
operator-selected 결정은 현재 고정 SHA 쌍에만 적용되는 별도 운용 결정이다.

`FS-03`은 향후 모델을 정식 승격하기 위한 정확도 기준이므로 `1/1/2 N`을
유지한다. `FS-06`은 현재 선택 모델의 실제 운용 중 이상을 검출하는 별도 기준이다.
2026-09-01 동일 zero-set task replay에서 느린 동작은 p95/p99/max
`0.951/1.181/1.722 N`, 정상 속도는 `1.072/1.336/2.096 N`이었고 false
CONTACT는 모두 0회였다. 이를 포함하면서 CONTACT ON 경계 `2.5 N`을 넘지 않도록
운용 한계를 `1.2/1.5/2.5 N`으로 고정했다. 한계 이내여도 CONTACT sample이 한
번이라도 있으면 `FS-06`은 실패한다.

근거 raw artifact는
`~/.ros/ft_fb_leaderarm/datasets/right_current_model_diag_20260901/` 아래의
`right_free_space_20260901_144955.npz`(SHA-256
`6ea93c27bed13a2e551c2e362aed947bd4020d2c3c98c21c7c8ef5a78d44587c`)와
`right_free_space_20260901_145052.npz`(SHA-256
`fca5ed7529827bb72e551023a94c04f00e871381a12f87f3d77f12e544a20633`)다.
두 artifact는 같은 `zero_set_id=tare_20260901_current_model_diag01`이므로 정식
독립 zero-set 모델 평가가 아니라 현재 모델의 운용 한계를 정하는 근거로만 쓴다.

## Contact observer gates

| ID | 합격 기준 | Evidence |
|---|---|---|
| `CO-01` | canonical `ContactObservation.contact_state`가 FREE=`0`, CONTACT=`1`을 publish | message/observer test와 topic capture |
| `CO-02` | `||F_contact_hat||_2` threshold와 Schmitt hold로 판정하며 valid/model-ready/fresh/sync fail-close 로직 유지 | observer unit/integration test |
| `CO-03` | leader arm 없이 observer 단독 launch 가능 | standalone launch test |
| `CO-04` | FREE false contact 0회, contact precision·recall과 onset/release latency가 확정 기준 통과 | 독립 same-clock interval 기반 contact report |
| `CO-05` | IL 수집과 policy inference가 동일한 canonical observation을 사용하고 publisher는 하나뿐임 | config hash와 두 모드의 graph/message report |

현재 detector 계약은 force residual ON/OFF `2.5/1.2 N`, CONTACT/FREE hold
`12/20 ms`다. 중간 구간은 상태를 유지하며 moment는 contact state 판정에 사용하지
않는다.

## Feedback leader gates

| ID | 합격 기준 | Evidence |
|---|---|---|
| `FB-01` | FREE·invalid·stale이면 reflected torque가 정확히 0이고 CONTACT일 때만 feedback 활성 | feedback analysis report |
| `FB-02` | CONTACT 시작 시 확정된 rise time과 최대 torque step 이내로 ramp-in | onset transient report |
| `FB-03` | torque clip 준수, leader pose step `<= 1 deg`, velocity reversal `<= 8 Hz`; 최종 진동 전달 기준 통과 | staged feedback report |
| `FB-04` | 기존 follower teleoperation을 유지하며 OFF→40%→100% evidence 승인 순서 준수 | authorization JSON과 원본 CSV hash |
| `FB-05` | 전용 GUI에서 feedback leader 제어와 IL episode 수집, raw/prediction/residual/state/model hash/timestamp/stage 저장 | `ft_il_episode_verify` report |

Canonical CONTACT에도 설정된 ramp-up을 적용한다. 합격 rise time과 torque step은
확정값을 evaluator에 명시하고 실측 report가 통과하기 전까지 승인하지 않는다.

## Open decisions

- 독립 contact ground truth 장치/기록 절차, precision·recall 및 onset/release latency 기준
- CONTACT 시작 최대 torque step과 feedback rise time
- leader 진동이 follower로 전달되는 정도의 metric과 합격 기준

미확정 값을 에이전트가 임의로 정해 통과시키지 않는다. 기준을 바꾸면 이 문서,
관련 설정·평가 코드·테스트를 같은 변경에서 갱신한다.

## Promotion order

software build/test → dataset/model → observer-only → feedback OFF → 40% → 100% →
IL test episode 순서로 승인한다. 실패·미확정 gate가 있으면 진행하지 않는다.
