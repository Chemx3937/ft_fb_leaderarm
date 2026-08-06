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
| dataset validator | 완료 | 실제 dataset에서 group split report 생성 필요 |
| 5개 ablation trainer | 완료 | 실제 데이터 학습 결과 없음 |
| 1 N validation/test gate | 완료 | 실제 승인 모델 없음 |
| 262.5 Hz benchmark gate | 완료 | target PC 실측 필요 |
| FT contact observer | 코드 초안 완료 | 실제 frame/sign/rate/contact SNR 검증 필요 |
| leader teleoperation | 코드 초안 완료 | feedback 방향과 안정성 실기 검증 필요 |
| OFF → 40% → 100% 승인 | 완료 | 각 단계 evidence는 아직 없음 |
| FT 실기 CSV analyzer | 완료 | 실제 threshold baseline 확정 필요 |
| IL recorder 연동 | interface 준비 | 기존 recorder/GUI와 end-to-end 검증 필요 |

## Phase 0: 안전·계약 확정

- [ ] 비상정지와 작업 공간 확인
- [ ] 오른팔 tool/payload를 고정하고 `payload_id` 확정
- [ ] impedance controller 설정 hash 확정
- [ ] AFT sensor frame과 sensor→TCP transform 실측
- [ ] 최대 허용 controlled-contact force 결정
- [ ] moment 예측/feedback을 사용할 경우 최대 허용 moment 오차 결정

## Phase 1: FT 센서 특성 확인

- [ ] 전원 ON 후 0/15/30/60/120분 warm-up drift 기록
- [ ] 같은 조건에서 hardware zero-set 10회 반복
- [ ] zero마다 여러 고정 자세 왕복 측정
- [ ] 재부팅 전후 반복성 확인
- [ ] 케이블 strain, 온도, EMI, overload 회복 확인
- [ ] [FT 센서 점검표](FTsensor_check_list.md)에 결과 기록

## Phase 2: 데이터 수집

- [ ] 최소 3개 독립 `zero_set_id` 확보
- [ ] 권장 8~10개 group을 시간대·재기동 조건으로 수집
- [ ] 각 group에서 저속/고속, 가속/감속, joint/Cartesian 동작 포함
- [ ] 모든 episode가 완전 무접촉인지 확인
- [ ] 접촉 직전에 반드시 collector stop
- [ ] tool/payload/controller가 바뀐 episode를 분리

## Phase 3: 데이터 검증

- [ ] `ft_free_space_validate` 실행
- [ ] rejected episode가 없는지 확인
- [ ] sample rate, gap, timestamp, frame 확인
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
