# TODO: Physical FT prediction, contact observer, feedback IL

## 최종 목표

오른팔 follower가 무접촉으로 움직일 때 물리 AFT가 측정하는 6축 free-space
wrench를 262.5 Hz로 예측한다. 무접촉 구간에서 다음을 만족하는 승인 모델을
만든다.

```text
contact_wrench = raw_physical_ft - predicted_free_space_wrench
aggregate p99 <= 1.0 N, every zero-set group p95 <= 1.0 N, hard max <= 2.0 N
```

이 residual을 canonical `ContactObservation`으로 발행하여 contact 판정,
leader force feedback, 모방학습 데이터 취득에 사용한다.

## 현재 구현 상태

`READY_FOR_EVIDENCE`는 검증 도구가 준비됐다는 뜻이며 실제 목표 합격은 아니다.

| 항목 | 상태 | 완료 기준 또는 남은 일 |
|---|---|---|
| Free-space 수집·검증·학습 GUI | `READY_FOR_EVIDENCE` | 독립 zero-set dataset과 실제 모델 report 필요 |
| robust force·262.5 Hz gate | `READY_FOR_EVIDENCE` | 새 held-out test와 runtime evidence 필요 |
| Contact observer·runtime/FREE 평가기 | `READY_FOR_EVIDENCE` | 실제 frame/sign/rate와 FREE residual 검증 필요 |
| Contact ground-truth 평가기 | `PARTIAL` | ground truth 절차와 precision·recall·latency 기준 확정 필요 |
| Feedback 분석·onset·단계 승인 | `READY_FOR_EVIDENCE` | rise time·torque step 기준과 OFF→40%→100% evidence 필요 |
| Feedback 진동 전달 검증 | `PARTIAL` | 사용자 요청 시 metric과 합격 기준을 정해 재개 |
| [Smooth teleop intent generator](stabilization_teleoperation/smooth_teleop_implementation.md) | `READY_FOR_EVIDENCE` | software filter/limiter 구현 완료, 동일 task hardware A/B 필요 |
| IL contact 계약 검증 | `READY_FOR_EVIDENCE` | collection/inference 양쪽의 실제 graph/message report 필요 |
| Feedback IL 통합 GUI·episode 검증 | `READY_FOR_EVIDENCE` | 새 session의 test episode와 PASS report 필요 |

## 다음 실행 작업

기록일: `2026-08-19 KST (+0900)`

| 순서 | 담당 | 다음 작업 | 완료 기준 | 상태 |
|---:|---|---|---|---|
| 1 | 사용자 | 시작 자세, tool/payload/controller/frame과 수집 안전 조건 재확인 | FS-01 계약과 현재 장비 상태 일치 | 완료 |
| 2 | 사용자 | FAST-only task 무접촉 episode 3개 수집 | 각 `accepted=true`, 접촉 0회 | 완료: `tare_02~04` |
| 3 | 사용자 | 다양한 궤적 10개와 최소 7개 추가 독립 zero group 수집 | payload/controller/frame 혼합 없는 dataset | 완료: 10 episodes/10 groups |
| 4 | 검증 | 확대 dataset validator와 5개 ablation 재학습 | FS-02~04 report 통과, 승인 모델 생성 | final13 FS-02/04 PASS, FS-03 FAIL |
| 5 | 사용자→검증 | observer-only FREE evidence 수집 | FS-05~06, CO-01~03 report 통과 | 대기 |
| 6 | 사용자→검증 | ground truth 절차·CO-04 기준 확정 후 controlled contact 평가 | CO-04 report 통과 | 대기 |
| 7 | 사용자→검증 | FB-02 기준 확정 후 feedback OFF→40%→100% 분석·승인 | FB-01, FB-02, FB-04 report 통과 | 대기 |
| 8 | 사용자→검증 | 통합 GUI로 새 IL test episode 저장·검증 | `ft_il_episode_verify` PASS | 대기 |
| 9 | 검증 | IL collection/inference에서 canonical contact 계약 확인 | CO-05 report 두 개 통과 | 대기 |
| 10 | 사용자→검증 | 보류한 진동 전달 기준 확정·검증 | FB-03 통과 | 사용자 요청 전 보류 |

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
- [ ] controlled-contact 전에 ground truth 절차와 CO-04 합격 기준 결정
- [ ] feedback 전에 CONTACT rise time과 최대 torque step 결정
- [ ] 사용자 요청 시 leader→follower 진동 전달 metric과 기준 결정

## Phase 1: FT 센서 특성 확인

- [x] 같은 조건의 hardware zero-set 3회 예비 측정 ([`FT-20260808-01`](problem/FT-20260808-01.md) OPEN)
- [x] 수정본을 `aft_zero_set2.py`로 분리하고 격리 회귀 test ([`FT-20260808-02`](problem/FT-20260808-02.md) MITIGATED)
- [x] zero-set2 launch를 단일 `sensor_name` 선택 방식으로 구성, 기본 `aft_sensor2`
- [x] 설정 zero 자세 재복귀 후 sensor2 zero-set2 수행 ([`FT-20260808-03`](problem/FT-20260808-03.md) CLOSED)
- [x] 같은 tare의 0/15/30/60/120분 post-zero drift 기록
  ([`FT-20260808-01`](problem/FT-20260808-01.md) OPEN)
- [x] AFT cold power-on 후 0/15/30/60/120분 warm-up drift 기록
  (2026-08-18/19 완료; 앞선 단조 Fz drift는 재현되지 않았으나 30분 gate 실패;
  [`FT-20260808-01`](problem/FT-20260808-01.md) 결과 참고)
- [x] 같은 조건에서 hardware zero-set 10회 반복
  ([`FT-20260808-01`](problem/FT-20260808-01.md) OPEN)
- [ ] zero마다 여러 고정 자세 왕복 측정
- [ ] 재부팅 전후 반복성 확인
- [ ] 케이블 strain, 온도, EMI, overload 회복 확인
- [x] 60초 정지 noise와 raw CAN 실제 갱신률 측정 ([`FT-20260808-04`](problem/FT-20260808-04.md) MITIGATED)
- [x] collector/observer 종료 guard 회귀 테스트와 collector Ctrl-C clean exit ([`FT-20260808-05`](problem/FT-20260808-05.md) CLOSED)
- [x] 현재 driver의 AFT sensor rate를 임시 500 Hz 운용 계약으로 확정
- [x] 공식 force STD에 맞춰 collector/observer zero gate를 0.40 N으로 변경
- [ ] 1000 Hz가 필요할 때 CAN drain/read와 configure command를 함께 수정·재측정
- [x] 현재까지의 rate/noise/gate/smoke 결과를 [FT 센서 점검표](FTsensor_check_list.md)에 기록

## Phase 2: 데이터 수집

- [x] `/tmp` 정지 smoke episode 저장과 262.5 Hz/gap/sync 검증
- [x] 최소 3개 독립 `zero_set_id` 확보
- [ ] 권장 8~10개 group을 시간대·재기동 조건으로 수집
- [ ] 각 group에서 저속/고속, 가속/감속, joint/Cartesian 동작 포함
- [x] CURRENT/SLOW를 제외하고 FAST부터 저장하는 gate 확인
- [x] `ft_fb_leaderarm` 전용 GUI에
  `collector START 성공 → CURRENT → SLOW → 접촉 전 STOP` 순서와 gate 사유를 표시
- [x] pilot 3개가 완전 무접촉인지 확인
- [ ] 접촉 직전에 반드시 collector stop
- [ ] tool/payload/controller가 바뀐 episode를 분리

## Phase 3: 데이터 검증

- [x] clean pilot에 `ft_free_space_validate` 실행
- [x] clean pilot의 3개 episode가 모두 accepted인지 확인
- [x] pilot sample rate, gap, timestamp, frame 확인
- [ ] 실제 sensor acquisition rate와 ROS publish rate 차이를 metadata/report에 반영
- [ ] 첫 동적 episode에서 raw와 4 ms causal 평균의 residual/contact latency 비교
- [ ] [ObserverInput-AFT 시간 정렬 체크리스트](FTsensor_check_list.md#observerinput-aft-시간-정렬-검증과-향후-개선) 측정과 개선 착수 조건 판정
- [x] train/validation/test가 `zero_set_id` 단위로 분리됐는지 확인
- [x] pilot dataset validation report 보존
- [x] final13의 13개 NPZ/JSON CRC·metadata·배열 검증
- [x] final13을 독립 zero group `7/3/3`으로 split하고 report 보존

## Phase 4: 학습·비교

- [x] 실제 수집·학습·observer 주기를 `262.5 Hz`로 확정
- [x] 정확도 gate를 aggregate p99 `1 N`, zero-set별 p95 `1 N`, hard max `2 N`으로 변경
- [x] model-only timing gate를 p99 `3.048 ms`, hard max `3.810 ms`로 확정
- [x] 기존 train13/targeted6가 262.5 Hz runtime 표본 계약과 일치함을 확인
- [ ] 방법 고정 후 새 독립 held-out에서 robust 정확도 gate 검증

- [x] pilot static linear 학습
- [x] pilot dynamic MLP 학습
- [x] pilot history MLP 학습
- [x] pilot history LSTM 학습
- [x] pilot history GRU 학습
- [x] pilot validation 최대/p95/RMSE 비교
- [x] pilot 선택 후 held-out test를 한 번만 평가
- [ ] validation/test 최대 force-vector 오차 1 N 이하 확인: `4.295/2.733 N`
- [x] model-only inference p99/최악 약 `38,300/20,500 Hz`
  (`0.0261/0.0489 ms`)
- [ ] `metadata.json`의 `approved=true` 확인: pilot은 `false`
- [x] final13의 5개 ablation 학습과 validation-only 모델 선택
- [x] final13 선택 `dynamic_mlp`만 held-out test 1회 평가
- [ ] final13 validation/test 최대 force-vector 오차 1 N 이하:
  `3.980/6.260 N`
- [x] final13 CPU model-only inference p99/최악 약 `41,900/32,400 Hz`
  (`0.02388/0.03088 ms`)
- [ ] final13 `metadata.approved=true`: `false`, runtime 승격 금지
- [x] final13 residual을 task/diverse, offset, 정지/이동, 속도/가속도, global lag로 분리 분석
- [x] validation-only causal acceleration smoothing과 짧은 history ablation 비교:
  단일 모델 최선 max `3.947 N`, FAIL
- [x] validation-only 단순 ensemble 비교: 최선 max `3.530 N`, FAIL
- [x] train group `3/5/7` learning curve: max `11.168/4.703/3.980 N`
- [ ] [재개 체크포인트](problem/FT-20260819-01.md#다음-session-재개-체크포인트)에
  정의한 독립 zero-set targeted train 6 groups 추가 수집
- [ ] 방법 확정 후 새 독립 zero-set held-out test 3개 이상 수집

## Phase 5: Observer-only 실기 검증

- [x] runtime rate/FREE residual과 contact ground-truth 평가기 구현
- [ ] 새 runtime hardware zero-set 수행
- [ ] `ft_observer_runtime_evaluate`로 유효 publish rate 262.5 Hz 확인
- [ ] 같은 report에서 FREE residual p95/p99 1 N, hard max 2 N 확인
- [ ] FREE false contact 0회 확인
- [ ] ground truth 절차와 precision·recall·onset/release latency 기준 확정
- [ ] `ft_contact_evaluate`로 controlled contact 검출/해제 기준 통과
- [ ] contact 방향과 base-frame 부호 확인

## Phase 6: 단계별 feedback 승인

- [x] feedback 분석·authorization·onset evaluator 구현
- [x] raw FK와 follower command 사이에 2차 intent generator와 velocity/acceleration/jerk limit 구현
- [x] raw/intent/final command CSV 분리 logging과 software 회귀 테스트 추가
- [ ] 동일 task에서 기존 contact-observer 대비 command/actual/joint smoothness A/B 검증
- [ ] CONTACT rise time과 최대 torque step 기준 확정
- [ ] feedback-OFF FREE CSV 3개 이상 수집
- [ ] feedback-OFF controlled-CONTACT CSV 수집
- [ ] OFF→40 analyzer가 `GO`인지 확인
- [ ] 40% authorization 생성
- [ ] 40%에서 방향·진동·pose jump 확인
- [ ] 40% FREE/CONTACT CSV 분석이 `GO`인지 확인
- [ ] 100% authorization 생성
- [ ] 100% 제한 운용 후 안정성 확인
- [ ] 사용자 요청 시 leader→follower 진동 전달 metric과 기준을 정해 FB-03 재개

## Phase 7: IL 적용

- [x] FT observer, 기존 UMI recorder와 Feedback Leader Arm GUI 통합 launch 구현
- [x] raw/prediction/residual/state/model hash/timestamp/stage 저장 계약 구현
- [x] 저장된 episode의 읽기 전용 verifier 구현
- [ ] 동일 observation topic의 publisher가 하나인지 실제 graph에서 확인
- [ ] 외부 UMI recorder와 policy의 262.5 Hz contact rate와 config hash 확인
- [ ] feedback OFF의 작은 test episode를 새 session에 저장하고 verifier PASS 확인
- [ ] IL collection과 policy inference에서 `ft_il_contact_verify` 각각 PASS
- [ ] 승인된 task 범위에서 본 IL 수집 시작

## Definition of Done

- 실제 장비의 독립 zero-set held-out test에서 p99 1 N, group p95 1 N, hard max 2 N 통과
- target PC에서 model p99 3.048 ms/hard max 3.810 ms와 ROS 262.5 Hz 모두 통과
- 장시간 FREE 실기에서 false contact 0회
- controlled contact의 precision·recall·onset/release latency 기준 통과
- feedback onset과 leader→follower 진동 전달 기준 통과
- OFF → 40% → 100% authorization chain이 raw CSV까지 재검증됨
- IL episode verifier와 collection/inference contact 계약이 모두 통과
