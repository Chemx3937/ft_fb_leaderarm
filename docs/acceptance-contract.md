# Acceptance contract

## Purpose

목표 구현과 단계 승인의 기준이다. 코드가 있어도 실측 evidence가 없으면 합격이 아니다.
운용은 `document/TODO_LIST.md`와 `document/flow.md`, 검증은
`docs/verification.md`를 따른다.

상태: `READY_FOR_EVIDENCE`=코드 있음·실측 필요, `PARTIAL`=일부만 있음,
`OPEN_DECISION`=기준 미확정, `NOT_IMPLEMENTED`=필수 구현 없음.

## Shared contract

- 오른팔 manipulator 시작 조인트 각도는
  `[5.5, 52.0, 112.0, 28.0, -107.0, -35.0] deg`다.
- 수집, 학습, observer는 `262.5 Hz` 계약을 공유한다.
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

## Free-space force prediction gates

| ID | 합격 기준 | Evidence | 현재 상태 |
|---|---|---|---|
| `FS-01` | 매 수집 session 시작 전 기준 자세 오차 `<= 1 deg`, 정지 및 hardware zero 확인 | collector metadata와 zero diagnostics | `READY_FOR_EVIDENCE` |
| `FS-02` | contact-free dataset, 독립 `zero_set_id` 3개 이상, payload/controller/frame 혼합 없음 | dataset validation report | `READY_FOR_EVIDENCE` |
| `FS-03` | validation과 선택 모델의 held-out test 모두 `max(e_force) <= 1 N` | `ablation_report.json` | `READY_FOR_EVIDENCE` |
| `FS-04` | target PC inference `p99 <= 3.048 ms`, `max <= 3.810 ms` | model runtime benchmark | `READY_FOR_EVIDENCE` |
| `FS-05` | observer ready 이후 측정 구간의 유효 publish rate `>= 262.5 Hz`, deadline miss·invalid·stale 0회 | 독립 runtime report | `READY_FOR_EVIDENCE` |
| `FS-06` | 독립 무접촉 동작에서 `max(||F_contact_hat||_2) <= 1 N` | observer-only FREE report | `READY_FOR_EVIDENCE` |
| `FS-07` | 전용 GUI에서 leader 제어, 안전 순서 강제, 수집, dataset 검증과 학습 실행·상태 확인 가능 | GUI integration test | `READY_FOR_EVIDENCE` |

모델 선택에는 validation만 사용하고 held-out test는 선택 모델에 한 번만 사용한다.
목표 미달 시 기존 ablation 결과를 비교한 뒤에만 입력이나 구조를 추가한다.

## Contact observer gates

| ID | 합격 기준 | Evidence | 현재 상태 |
|---|---|---|---|
| `CO-01` | canonical `ContactObservation.contact_state`가 FREE=`0`, CONTACT=`1`을 publish | message/observer test와 topic capture | `READY_FOR_EVIDENCE` |
| `CO-02` | `||F_contact_hat||_2` threshold와 Schmitt hold로 판정하며 valid/model-ready/fresh/sync fail-close 로직 유지 | observer unit/integration test | `READY_FOR_EVIDENCE` |
| `CO-03` | leader arm 없이 observer 단독 launch 가능 | standalone launch test | `READY_FOR_EVIDENCE` |
| `CO-04` | FREE false contact 0회, contact precision·recall과 onset/release latency가 확정 기준 통과 | 독립 same-clock interval 기반 contact report | `PARTIAL` |
| `CO-05` | IL 수집과 policy inference가 동일한 canonical observation을 사용하고 publisher는 하나뿐임 | 두 경로의 end-to-end topic capture | `PARTIAL` |

## Feedback leader gates

| ID | 합격 기준 | Evidence | 현재 상태 |
|---|---|---|---|
| `FB-01` | FREE·invalid·stale이면 reflected torque가 정확히 0이고 CONTACT일 때만 feedback 활성 | feedback analysis report | `READY_FOR_EVIDENCE` |
| `FB-02` | CONTACT 시작 시 확정된 rise time과 최대 torque step 이내로 ramp-in | onset transient report | `NOT_IMPLEMENTED` |
| `FB-03` | torque clip 준수, leader pose step `<= 1 deg`, velocity reversal `<= 8 Hz`; 최종 진동 전달 기준 통과 | staged feedback report | `PARTIAL` |
| `FB-04` | 기존 follower teleoperation을 유지하며 OFF→40%→100% evidence 승인 순서 준수 | authorization JSON과 원본 CSV hash | `READY_FOR_EVIDENCE` |
| `FB-05` | 전용 GUI에서 feedback leader 제어와 IL episode 수집, raw/prediction/residual/state/model hash/timestamp/stage 저장 | GUI/recorder end-to-end report | `PARTIAL` |

현재 CONTACT 순간 `tau_fb_contact_scale_`이 `1.0`이 되므로 `FB-02` 통과 전에는
contact-on feedback을 승인하지 않는다.

## Open decisions

- 독립 contact ground truth 장치/기록 절차, precision·recall 및 onset/release latency 기준
- CONTACT 시작 최대 torque step과 feedback rise time
- leader 진동이 follower로 전달되는 정도의 metric과 합격 기준

미확정 값을 에이전트가 임의로 정해 통과시키지 않는다. 기준을 바꾸면 이 문서,
관련 설정·평가 코드·테스트를 같은 변경에서 갱신한다.

## Promotion order

software build/test → dataset/model → observer-only → feedback OFF → 40% → 100% →
IL test episode 순서로 승인한다. 실패·미확정 gate가 있으면 진행하지 않는다.
