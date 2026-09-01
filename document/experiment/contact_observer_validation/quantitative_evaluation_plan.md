# Contact observer 정량 평가 실험 계획

## 1. 목적과 현재 상태

현재 free-space wrench 모델과 Schmitt detector가 실제 외력 접촉을 얼마나 정확하게
구분하는지 독립 ground truth로 평가한다. 저장된 모방학습 contact state는 이전
observer 출력이므로 최종 물리 정답으로 사용하지 않는다.

이 문서의 observer 값은 2026-09-01 runtime 설정에 적용했다. 독립 test 결과가
`CO-04`의 확정 기준을 통과했다는 뜻은 아니며, controlled-contact 검증은 남아 있다.

## 2. 육하원칙

- 누가: 승인된 실험자가 로봇을 운용하고, observer와 독립 외력 센서가 각각 예측과
  ground truth를 기록한다.
- 언제: observer-only, feedback OFF 상태에서 software 검증과 센서 보정을 마친 뒤
  수행한다.
- 어디서: 실제 logistic-box 접촉 작업 공간에서, 접촉 대상 또는 접촉판을 독립
  load cell에 고정한 상태로 수행한다.
- 무엇을: sample/event contact 정확도, FREE 오검출, contact 누락과 onset/release
  latency를 측정한다.
- 어떻게: 같은 clock으로 observer 출력과 독립 센서를 동시에 수집하고, episode를
  시간순으로 평가한다.
- 왜: 모방학습 데이터와의 일치도가 아니라 실제 외력 기준의 contact 성능을
  정량화하기 위해서다.

## 3. 평가할 observer 판정 기준

Force residual은 다음과 같다.

```text
e_force(t) = ||F_sensor(t) - F_free_hat(t)||_2
```

현재 runtime 기준은 다음과 같다.

```yaml
force_on_n: 3.0
force_off_n: 1.2
contact_hold_ms: 20.0
free_hold_ms: 20.0
```

- FREE에서 `e_force >= 3.0 N`이 20 ms 이상 지속되면 CONTACT로 전환한다.
- CONTACT에서 `e_force <= 1.2 N`이 20 ms 이상 지속되면 FREE로 전환한다.
- `1.2 N < e_force < 3.0 N`에서는 현재 상태를 유지한다.
- invalid, model-not-ready, stale 또는 sync 실패 시 기존 fail-close 계약을 유지한다.

이전 runtime 값 `2.0/1.2 N`, `8/20 ms`에서 `force_on_n`과
`contact_hold_ms`를 `2.5 N`, `12 ms`로 변경해 운용했다. 2026-09-01 무접촉
logistic-box task 2회에서 false CONTACT가 재현되어 현재 임시값을 `3.0 N`,
`20 ms`로 변경했다.

### 후보 선택 근거

102개 모방학습 episode의 저장 state와 비교한 offline replay 결과다. 이는 threshold
선정 참고값이며 독립 ground-truth 성능이나 `CO-04` 승격 evidence가 아니다.

| 지표 | 이전 `2.0/1.2 N, 8/20 ms` | 변경 전 `2.5/1.2 N, 12/20 ms` | 현재 임시 `3.0/1.2 N, 20/20 ms` |
|---|---:|---:|---:|
| Accuracy | 86.36% | 89.06% | 89.66% |
| Balanced accuracy | 81.17% | 82.39% | 82.16% |
| CONTACT precision | 77.28% | 88.84% | 93.86% |
| CONTACT recall | 69.91% | 67.93% | 65.91% |
| CONTACT F1 | 73.41% | 76.99% | 77.44% |
| False contact activation | 644 | 274 | 190 |
| Event precision | 46.65% | 83.67% | 95.81% |
| Event recall | 97.12% | 95.19% | 94.23% |
| Onset latency p95 | 18.83 ms | 37.73 ms | 217.06 ms |
| Release latency p95 | 28.49 ms | 8.85 ms | 5.85 ms |

안정 FREE 구간의 residual p99가 `2.424 N`이므로 `2.0 N` ON 기준은 정상 FREE
오차에도 활성화되기 쉽다. 추가 무접촉 task 2회의 replay에서 `2.5 N / 40 ms`와
`3.0 N / 20 ms`가 모두 false CONTACT 0회였고, 더 짧은 hold에서 같은 pseudo-label
event recall을 보인 `3.0 N / 20 ms`를 임시 선택했다. 이 값은 독립 ground truth가
아니므로 실제 contact recall이나 `CO-04` 통과를 증명하지 않는다. 자세한 free-space 수치는
[free-space 검증 결과](../free_space_wrench_model_validation/README.md)를 따른다.

## 4. 독립 ground truth 구성

### 4.1 권장 장치

여러 방향의 접촉을 평가할 수 있도록 logistic box 또는 접촉판을 3축/6축 load cell에
고정한다.

```text
robot end-effector -> box/contact plate -> independent load cell -> fixed frame
```

Observer는 기존 AFT wrench를 입력으로 사용하고, ground truth는 물체 측 load cell만
사용한다. 같은 AFT wrench나 observer residual로 ground truth를 만들지 않는다.

### 4.2 보정과 동기화

1. 물체 무게를 포함한 상태로 tare하고 무접촉 baseline을 기록한다.
2. 알려진 힘으로 필요한 각 축의 scale, sign, saturation을 확인한다.
3. 무접촉 노이즈의 p99/p99.9와 hard max를 측정한다.
4. 독립 센서는 가능하면 `262.5 Hz` 이상으로 취득한다.
5. observer와 ground truth를 같은 PC/ROS clock으로 timestamp한다. 별도 clock이면
   공통 hardware trigger로 offset과 동기화 불확도를 측정한다.
6. raw 독립 wrench와 contact interval을 모두 보존한다.

### 4.3 Ground-truth CONTACT 기준

사용자가 정한 최소 의미 외력 `F_required`와 센서 FREE 노이즈를 이용한다.

```text
F_gt(t) = ||F_external(t) - F_external_baseline||_2
GT ON   = max(F_required, FREE noise p99.9 + sensor uncertainty margin)
GT OFF  = GT ON보다 낮고 FREE noise보다 높은 값
```

`GT ON`, `GT OFF`와 ground-truth hold는 독립 센서 측정 후 확정하여 원본 데이터와
함께 기록한다. Observer의 `3.0/1.2 N`을 그대로 복사하지 않는다.

## 5. 데이터 수집 계획

### 5.1 데이터 분리

- Pilot/validation session 1개: ground-truth 기준과 observer 후보 조정에만 사용한다.
- Held-out test session 3개 이상: 설정을 고정한 뒤 한 번만 최종 평가한다.
- session은 서로 다른 `zero_set_id`를 사용하고 실제 runtime과 같은 payload,
  controller, sensor frame 계약을 유지한다.
- test 결과를 보고 값을 다시 조정하면 기존 test는 validation으로 이동하고 새 test를
  수집한다.

### 5.2 FREE-only 조건

- 정지, 느린 동작, 정상 운용 속도 동작
- 큰 가감속, 관절 방향 전환, 여러 작업 자세
- 물체 가까이 접근하지만 닿지 않는 동작

Ground truth가 전 구간 FREE인지 독립 센서로 확인한다. 결과에는 총 FREE 시간,
false CONTACT sample/event 수와 시간당 activation을 기록한다. 0 event를 관측했을 때
95% 신뢰수준의 발생률 상한은 대략 `3 / 측정시간[h]`이므로 요구 false-event-rate에
맞춰 시험 시간을 정한다.

### 5.3 CONTACT 조건

- 느림/정상/최대 운용 접근 속도
- 여러 접근 방향
- ground-truth ON 부근의 약한 접촉, 일반 접촉, 강한 접촉
- 짧은 tap, 1~3초 유지, 밀기/미끄러짐, 반복 접촉과 해제

각 episode는 `접촉 전 FREE -> 접근 -> CONTACT -> 해제 -> 접촉 후 FREE` 순서로
구성한다. 최종 test는 조건별 반복을 포함해 aggregate 100~300 contact event를
목표로 하며, 실제 요구 신뢰구간에 따라 늘린다.

### 5.4 안전 순서

1. software test
2. 센서 보정과 고정 상태 확인
3. observer-only, feedback OFF
4. 저속/저힘 pilot
5. 승인된 범위에서 속도와 힘을 단계적으로 증가

로봇 이동, FT zero-set과 데이터 수집은 사용자 승인 없이 실행하지 않는다.

## 6. 저장 형식과 순차 재생

실시간 시험에서는 로봇 동작 중 observer 출력과 독립 센서를 동시에 기록한다.
오프라인 순차 재생은 로봇을 다시 움직이는 작업이 아니라, 기록된 joint/wrench를
timestamp 순으로 observer에 입력하는 작업이다. 모델 history, Schmitt state와 hold를
유지해야 하므로 sample을 섞거나 각 step을 독립 추론하지 않는다.

원본 episode에는 최소한 다음을 보존한다.

- timestamp, joint position/velocity
- raw AFT wrench, predicted free-space wrench, residual force norm
- observer contact state, valid, model-ready, stale/sync 상태
- 독립 센서 raw wrench와 ground-truth state/interval
- model/config SHA-256, `zero_set_id`, payload/controller/frame 정보

기존 evaluator 입력은 다음 CSV 두 개다.

```text
observations.csv:
t_s,observer_contact_state,observer_valid,observer_model_ready

ground_truth.csv:
start_s,end_s
```

두 CSV는 같은 clock을 사용하고 contact interval은 겹치거나 맞닿지 않게 정렬한다.
실행 명령은 [CO-04 ground-truth 평가 절차](../../../docs/verification.md#contact-ground-truth-evidence)를
따른다.

## 7. 정량 지표와 판정

### 7.1 Sample 지표

```text
TP: ground truth CONTACT, observer CONTACT
FP: ground truth FREE,    observer CONTACT
FN: ground truth CONTACT, observer FREE
TN: ground truth FREE,    observer FREE

accuracy             = (TP + TN) / N
precision            = TP / (TP + FP)
recall               = TP / (TP + FN)
specificity          = TN / (TN + FP)
F1                   = 2 * precision * recall / (precision + recall)
balanced accuracy    = (recall + specificity) / 2
```

FREE 비율이 높으면 accuracy가 과대평가되므로 precision, recall, F1과 balanced
accuracy를 함께 보고한다.

### 7.2 Event와 latency 지표

- Event precision/recall과 완전히 누락된 contact event 수
- FREE false activation event 수와 시간당 발생률
- onset latency = `observer CONTACT 시각 - ground-truth CONTACT 시작 시각`
- release latency = `observer FREE 시각 - ground-truth CONTACT 종료 시각`
- onset/release의 p50, p95, max와 조기/지연 판정 수
- 실제 한 contact에서 여러 번 전환한 chatter와 contact fragmentation 수

각 episode와 속도/방향/힘 조건별 결과, 전체 aggregate를 모두 보고한다. 신뢰구간은
시간 상관이 있는 sample이 아니라 episode 단위 bootstrap으로 계산한다.

### 7.3 Invalid 처리

정상 `valid/ready/fresh/sync` sample의 detector 성능과 invalid를 포함한 end-to-end
성능을 분리한다. 실제 CONTACT 중 fail-close로 FREE가 출력된 구간은 운영 관점의
contact 누락으로 별도 집계한다.

### 7.4 합격 기준

현재 계약상 다음은 고정이다.

- FREE false CONTACT sample `0`
- invalid sample `0`, model-not-ready sample `0`
- 모든 ground-truth onset/release 검출

다음 값은 아직 프로젝트의 미확정 결정이며 시험 전에 사용자가 승인해야 한다.

```text
min_precision             = TBD
min_recall                = TBD
max_onset_latency_ms      = TBD
max_release_latency_ms    = TBD
```

기존 `ft_contact_evaluate`는 TP/FP/FN/TN 기반 precision/recall, false activation,
각 event latency와 최대 latency를 계산하고 위 기준으로 PASS/FAIL을 판정한다.
Accuracy, specificity, F1, balanced accuracy, p50/p95, event precision/recall과
episode bootstrap 신뢰구간은 현재 evaluator 출력에 없으므로 최종 실험 전에 평가기
확장 또는 별도 읽기 전용 분석이 필요하다.

## 8. 최종 산출물

- 보정 기록, ground-truth threshold와 clock 동기화 불확도
- 변경 불가능하게 보존한 원본 episode/CSV와 SHA-256
- episode별 및 aggregate 지표/plot
- `contact_evaluation_<timestamp>.json`
- 사용한 model/config hash와 observer 값
- PASS/FAIL 및 실패한 조건 목록

현재 threshold는 운용 기준으로 선택됐지만, 실측 evidence가 없거나 합격 기준이
미확정이면 `CO-04`를 PASS로 판정하지 않는다.
