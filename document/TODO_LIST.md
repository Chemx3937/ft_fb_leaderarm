# TODO: Physical FT free-space wrench model

## 최종 목표

오른팔 follower가 무접촉으로 움직일 때 물리 AFT가 측정하는 6축 free-space
wrench를 262.5 Hz로 예측한다. 무접촉 구간에서 다음을 만족하는 승인 모델을
만든다.

```text
contact_wrench = raw_physical_ft - predicted_free_space_wrench
max_t ||Fraw(t) - Fprediction(t)||2 <= 1.0 N
```

이 residual을 canonical `ContactObservation`으로 발행하여 contact 판정,
leader force feedback, 모방학습 데이터 취득에 사용한다.

## 현재 구현 상태

| 항목 | 상태 | 완료 기준 또는 남은 일 |
|---|---|---|
| FT free-space collector | 코드 초안 완료 | 실제 AFT/robot에서 episode 저장 확인 필요 |
| 고정 자세 zero 검증 | 코드 초안 완료 | 장시간 drift와 반복 tare 실측 필요 |
| SBC zero-set2 launch | 실제 sensor2 tare/종료 PASS | legacy `aft_zero_set` 교착은 보존, 단일 센서는 zero-set2 사용 |
| dataset validator | 완료 | 실제 dataset에서 group split report 생성 필요 |
| 5개 ablation trainer | 완료 | 실제 데이터 학습 결과 없음 |
| 1 N validation/test gate | 완료 | 실제 승인 모델 없음 |
| 262.5 Hz benchmark gate | 완료 | target PC 실측 필요 |
| ObserverInput-AFT 시간 정렬 | latest-only 3 ms gate | 실측 후 필요 시 nearest-state pairing 구현 |
| FT contact observer | 코드 초안 완료 | 실제 frame/sign/rate/contact SNR 검증 필요 |
| leader teleoperation | 코드 초안 완료 | feedback 방향과 안정성 실기 검증 필요 |
| OFF → 40% → 100% 승인 | 완료 | 각 단계 evidence는 아직 없음 |
| FT 실기 CSV analyzer | 완료 | 실제 threshold baseline 확정 필요 |
| IL recorder 연동 | interface 준비 | 기존 recorder/GUI와 end-to-end 검증 필요 |

## 다음 실행 작업

기록 시각: `2026-08-11 12:17:16 KST (+0900)`

| 순서 | 담당 | 다음 작업 | 완료 기준 | 상태 |
|---:|---|---|---|---|
| 1 | 사용자 | 현재 driver/controller/AFT와 Chrony 상태를 중복 실행 없이 확인 | Chrony `GO`, ObserverInput/AFT topic 정상 | 완료: 두 topic 1 publisher/약 1000 Hz, Chrony bound `0.041132 ms`, `GO` |
| 2 | 사용자 | AFT를 이번에 시작한 뒤 sensor2 500 Hz one-shot 실행 | 명령 1회 정상 발행 | 완료: subscriber 1개, `Int32(data=500)` 1회 발행 |
| 3 | 사용자 | feedback-OFF teleop startup `ALIGN`; 자세가 다를 때만 `INIT POSE → REALIGN` | follower fixed zero pose, Teleop `IDLE` | 완료: follower 약 `[5.45,51.88,112.20,27.96,-106.77,-34.95]°`, startup `IDLE` |
| 4 | 사용자 | cable/tool 무접촉 상태에서 `aft_zero_set2` 실행 | zero-set2 clean exit | 완료: 100 samples, hardware tare acknowledged, clean exit |
| 5 | 사용자 | FT 전용 GUI를 고유 `zero_set_id`로 실행하고 첫 SLOW 무접촉 episode 수집 | `20~30초`, `accepted=true`, 접촉 0회 | 정식 재수집 필요: 첫 파일은 품질 통과했으나 `74.757초` |
| 6 | 사용자→검증 | 저장된 `.npz/.json` 경로 확인 및 전달 | sample/sync/metadata 검증 가능 | 현재 파일 검증 완료: `262.504 Hz`, sync max `2.961 ms`, metadata 일치 |

전체 순서와 명령은 [free-space wrench 데이터 수집 가이드](free_space_wrench_data_collection.md)를
따른다. 첫 실제 episode 검증이 끝나기 전에는 여러 episode나 FAST 동작으로 확대하지
않는다.

## Phase 0: 안전·계약 확정

- [ ] 비상정지와 작업 공간 확인
- [ ] [PC-SBC 시계 동기화와 FT sample 시간 정렬](timing_sync.md)을 구분해 검증
- [ ] 오른팔 tool/payload를 고정하고 `payload_id` 확정
- [ ] impedance controller 설정 hash 확정
- [ ] AFT sensor frame과 sensor→TCP transform 실측
- [ ] 최대 허용 controlled-contact force 결정
- [ ] moment 예측/feedback을 사용할 경우 최대 허용 moment 오차 결정

## Phase 1: FT 센서 특성 확인

- [x] 같은 조건의 hardware zero-set 3회 예비 측정 (`FT-20260808-01` OPEN)
- [x] 수정본을 `aft_zero_set2.py`로 분리하고 격리 회귀 test (`FT-20260808-02` MITIGATED)
- [x] zero-set2 launch를 단일 `sensor_name` 선택 방식으로 구성, 기본 `aft_sensor2`
- [x] 설정 zero 자세 재복귀 후 sensor2 zero-set2 수행 (`FT-20260808-03` CLOSED)
- [ ] 전원 ON 후 0/15/30/60/120분 warm-up drift 기록
- [ ] 같은 조건에서 hardware zero-set 10회 반복
- [ ] zero마다 여러 고정 자세 왕복 측정
- [ ] 재부팅 전후 반복성 확인
- [ ] 케이블 strain, 온도, EMI, overload 회복 확인
- [x] 60초 정지 noise와 raw CAN 실제 갱신률 측정 (`FT-20260808-04` MITIGATED)
- [x] collector/observer 종료 guard 회귀 테스트와 collector Ctrl-C clean exit (`FT-20260808-05` CLOSED)
- [x] 현재 driver의 AFT sensor rate를 임시 500 Hz 운용 계약으로 확정
- [x] 공식 force STD에 맞춰 collector/observer zero gate를 0.40 N으로 변경
- [ ] 1000 Hz가 필요할 때 CAN drain/read와 configure command를 함께 수정·재측정
- [x] 현재까지의 rate/noise/gate/smoke 결과를 [FT 센서 점검표](FTsensor_check_list.md)에 기록

## Phase 2: 데이터 수집

- [x] `/tmp` 정지 smoke episode 저장과 262.5 Hz/gap/sync 검증
- [ ] 최소 3개 독립 `zero_set_id` 확보
- [ ] 권장 8~10개 group을 시간대·재기동 조건으로 수집
- [ ] 각 group에서 저속/고속, 가속/감속, joint/Cartesian 동작 포함
- [ ] 첫 episode를 zero pose에서 시작한 뒤 leader CURRENT 전환 transient 포함 확인
- [x] `ft_fb_leaderarm` 전용 GUI에
  `collector START 성공 → CURRENT → SLOW → 접촉 전 STOP` 순서와 gate 사유를 표시
- [ ] 모든 episode가 완전 무접촉인지 확인
- [ ] 접촉 직전에 반드시 collector stop
- [ ] tool/payload/controller가 바뀐 episode를 분리

## Phase 3: 데이터 검증

- [ ] `ft_free_space_validate` 실행
- [ ] rejected episode가 없는지 확인
- [ ] sample rate, gap, timestamp, frame 확인
- [ ] 실제 sensor acquisition rate와 ROS publish rate 차이를 metadata/report에 반영
- [ ] 첫 동적 episode에서 raw와 4 ms causal 평균의 residual/contact latency 비교
- [ ] [ObserverInput-AFT 시간 정렬 체크리스트](FTsensor_check_list.md#observerinput-aft-시간-정렬-검증과-향후-개선) 측정과 개선 착수 조건 판정
- [ ] train/validation/test가 `zero_set_id` 단위로 분리됐는지 확인
- [ ] dataset validation report 보존

## Phase 4: 학습·비교

- [ ] static linear 학습
- [ ] dynamic MLP 학습
- [ ] history MLP 학습
- [ ] history LSTM 학습
- [ ] history GRU 학습
- [ ] validation 최대/p95/RMSE 비교
- [ ] 선택 후 held-out test는 한 번만 평가
- [ ] validation/test 최대 force-vector 오차 1 N 이하 확인
- [ ] inference p99 3.048 ms, max 3.810 ms 이하 확인
- [ ] `metadata.json`의 `approved=true` 확인

## Phase 5: Observer-only 실기 검증

- [ ] 새 runtime hardware zero-set 수행
- [ ] `/contact_observer/right/observation` 262.5 Hz 확인
- [ ] FREE 동작에서 residual force 최대 1 N 확인
- [ ] FREE false contact 0회 확인
- [ ] 제한된 controlled contact에서 검출/해제 확인
- [ ] contact 방향과 base-frame 부호 확인

## Phase 6: 단계별 feedback 승인

- [ ] feedback-OFF FREE CSV 3개 이상 수집
- [ ] feedback-OFF controlled-CONTACT CSV 수집
- [ ] OFF→40 analyzer가 `GO`인지 확인
- [ ] 40% authorization 생성
- [ ] 40%에서 방향·진동·pose jump 확인
- [ ] 40% FREE/CONTACT CSV 분석이 `GO`인지 확인
- [ ] 100% authorization 생성
- [ ] 100% 제한 운용 후 안정성 확인

## Phase 7: IL 적용

- [ ] FT observer와 기존 IL recorder source 연결 확인
- [ ] raw physical FT와 predicted/contact wrench가 모두 저장되는지 확인
- [ ] 동일 observation topic을 두 observer가 동시에 발행하지 않는지 확인
- [ ] 작은 test episode를 저장하고 timestamp/frame/schema 검증
- [ ] 승인된 task 범위에서 본 IL 수집 시작

## Definition of Done

- 실제 장비의 독립 zero-set held-out test에서 최대 force-vector 오차 ≤ 1 N
- target PC에서 model deadline과 ROS 262.5 Hz 모두 통과
- 장시간 FREE 실기에서 false contact 0회
- controlled contact의 방향·검출·해제가 일관됨
- OFF → 40% → 100% authorization chain이 raw CSV까지 재검증됨
- IL episode에 physical raw/prediction/contact residual과 model identity가 보존됨
