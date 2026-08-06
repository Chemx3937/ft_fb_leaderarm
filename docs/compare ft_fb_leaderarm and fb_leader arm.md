# `ft_fb_leaderarm`과 `fb_leaderarm` 비교

## 결론

두 패키지의 오른팔 single-impedance leader teleoperation 제어 구조는 같다.
핵심 차이는 contact wrench의 출처다.

```text
fb_leaderarm    : JTS 기반 wrench - JTS free-space prediction
ft_fb_leaderarm : physical FT raw wrench - physical FT free-space prediction
```

따라서 leader/follower mapping과 feedback torque 적용 구조는 유지되지만,
모델의 데이터·zero-set·frame·drift 계약은 서로 호환되지 않는다.

## 상세 비교

| 항목 | `fb_leaderarm` V2 | `ft_fb_leaderarm` | 결과적으로 달라지는 점 |
|---|---|---|---|
| 목적 | JTS 추정 wrench의 free-space 성분 제거 | 물리 AFT의 free-space 성분 제거 | FT 모델은 실제 센서 noise·drift까지 학습해야 함 |
| 대상 팔 | 오른팔 중심 V2 workflow | 오른팔만 허용 | FT dataset에 왼팔 데이터를 섞을 수 없음 |
| raw wrench | `/bae_r/observer_input.measured_wrench` 또는 `/bae_r/F_e` 계열 | `/aft_sensor2/wrench` | FT 장착, 케이블, 온도, tare가 성능에 직접 영향 |
| robot state | `/bae_r/observer_input` | `/bae_r/observer_input` | q, dq, current pose와 source timestamp 계약은 유지 |
| 학습 입력 | q, dq, qdd와 V2 보정 문맥 | q, dq, causal qdd만 | contact leakage 가능성이 있는 measured wrench/task error를 입력에서 제외 |
| 학습 target | JTS free-space wrench | sensor-frame physical FT 6축 wrench | 두 모델 artifact는 교환할 수 없음 |
| zero | V2 runtime baseline/residual 보정 포함 | 고정 초기 자세 hardware AFT zero-set 필수 | online bias가 실제 contact를 흡수할 위험은 줄고 zero 반복성 의존은 증가 |
| runtime 보정 | prediction LPF, residual bias, task correction 존재 | 직접 예측, runtime residual bias 없음 | 지연은 작지만 순간 noise가 residual에 직접 나타남 |
| 모델 후보 | 기존 V2 학습 후보 | static linear, dynamic/history MLP, LSTM, GRU | FT에서는 5개 후보를 동일 group split으로 비교 |
| 모델 주기 | inference worker와 cached observation 구조 | 262.5 Hz 직접 inference/publish | FT 모델은 3.810 ms deadline을 통과해야 함 |
| 최대 force 오차 | V2 자체 승인 계약 | validation/test 모두 force-vector 최대 1 N | 평균 RMSE가 좋아도 한 sample이 1 N을 넘으면 불승인 |
| contact detector | V2 canonical detector | force norm 2.0/1.2 N, 8/20 ms 기본값 | FT threshold는 실제 drift/contact SNR 측정 후 조정 필요 |
| moment contact | 기본 운용에서 제한적 | contact state는 force norm만 사용 | 순수 moment 접촉은 검출하지 못할 수 있음 |
| observer 출력 | `ContactObservation`, base frame | 동일 | leader teleop과 IL recorder의 소비 인터페이스 유지 |
| feedback 계산 | contact wrench → leader Jacobian transpose torque | 동일 | wrench가 같다면 feedback 적용 메커니즘은 동일 |
| feedback OFF | observer 구독 유지, gain 0 | 동일 | OFF에서도 contact state, valid, stale을 계속 관찰 |
| gain 단계 | 기존 V2 다단계 | OFF → 40% → 100% | 20%와 80% 단계는 FT workflow에서 사용하지 않음 |
| 단계 승인 | V2 evidence/authorization | FT CSV analyzer report와 이전 승인 hash 결합 | raw CSV가 분석 후 바뀌면 다음 launch가 거부됨 |
| 실기 분석 | V2 Observer NPZ/Leader CSV 계약 | 확장된 FT Leader CSV 하나를 분석 | 실제 gain stage, false contact, 최대 force/torque, pose jump, vibration 지표 자동 계산 |
| 원본 코드 영향 | 기준 패키지 | 별도 패키지 | `fb_leaderarm` 원본은 수정하지 않음 |

## 같은 부분

- leader Dynamixel 제어
- leader/follower joint mapping
- follower command 발행
- gravity compensation과 damping
- CURRENT → SLOW → FAST keyboard FSM
- Jacobian transpose 기반 reflected torque
- per-joint gain, clip, slew limit
- motion gate, passivity gate, stale fail-close
- canonical `ContactObservation` 구독 및 contact-state gate

## FT라서 의도적으로 다른 부분

### Hardware zero-set

AFT zero-set은 지정 초기 자세, 동일 tool/payload, 완전 정지, 무접촉 상태에서만
수행한다. 같은 자세에서 zero 직후 0에 가까운지만 확인해서는 반복성을 판단할
수 없다. 여러 검증 자세를 왕복한 측정값을 독립 zero-set group 사이에서
비교해야 한다.

### 모델 입력 제한

`task_error`, measured wrench, raw/measured joint torque는 runtime feature로
사용하지 않는다. 이 값이 실제 contact를 암시하면 모델이 contact까지
free-space wrench로 예측하여 residual을 지울 수 있기 때문이다.

### 승인 의미

40% authorization은 feedback-OFF 실기 로그 분석을 통과한 모델만 허용한다.
100% authorization은 같은 모델의 40% 실기 로그 분석과 40% authorization을
모두 다시 검증한다. 자동 analyzer가 feedback 방향의 물리적 옳고 그름까지
판단할 수는 없으므로 방향은 운영자가 직접 확인하고 attestation한다.

## 관련 문서

- [전체 흐름](flow.md)
- [목표와 TODO](TODO_LIST.md)
- [학습 architecture](base_architecture.md)
- [실행 명령](command.md)
- [FT 센서 점검](FTsensor_check_list.md)
- [실패 기록](failure_log.md)
