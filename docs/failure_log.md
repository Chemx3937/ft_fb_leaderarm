# FT free-space wrench failure log

## 목적

실패를 덮어쓰거나 기억에 의존하지 않고, 같은 조건의 재발 여부와 수정 효과를
추적한다. 문제를 기록할 때는 육하원칙과 artifact 경로를 반드시 포함한다.

## 분류 코드

| 코드 | 분류 | 예시 |
|---|---|---|
| `SENSOR` | AFT hardware/통신 | CAN drop, saturation, EMI spike |
| `ZERO` | tare/warm-up | zero 불안정, 장시간 drift |
| `SYNC` | timestamp/rate | FT-state sync 초과, source stale |
| `DATA` | collector/dataset | 접촉 혼입, gap, metadata mismatch |
| `MODEL` | 학습/정확도 | validation/test 1 N 초과, overfit |
| `RUNTIME` | inference/ROS | 3.810 ms deadline miss, 262.5 Hz 미달 |
| `CONTACT` | detector | false positive/negative, 해제 지연 |
| `FEEDBACK` | leader 반력 | 방향 반대, 진동, pose jump, clip 지속 |
| `ROBOT` | driver/controller | impedance controller fault, joint stale |
| `OPS` | 운용 절차 | 잘못된 payload ID, 접촉 중 zero-set |

## 상태

- `OPEN`: 원인 미확정 또는 수정 전
- `MITIGATED`: 임시 회피 적용
- `VERIFY`: 수정 후 재실험 대기
- `CLOSED`: 같은 조건의 재실험에서 해결 확인
- `WONT_FIX`: 범위 밖이며 이유를 기록함

## 실패 목록

| ID | 날짜 | 분류 | 단계 | 증상 | 상태 | 상세 항목 |
|---|---|---|---|---|---|---|
| `FT-20260808-01` | 2026-08-08 | `ZERO` | Phase 1 | Fz tare 중심값 차이와 정지 noise로 zero gate 간헐 실패 | `OPEN` | [상세](#ft-20260808-01-fz-zero-반복성과-정지-noise) |
| `FT-20260808-02` | 2026-08-08 | `ZERO` | Phase 1 | SBC legacy zero-set node 2개가 bias 후 종료되지 않음 | `MITIGATED` | [상세](#ft-20260808-02-sbc-zero-set-node-종료-교착) |
| `FT-20260808-03` | 2026-08-08 | `OPS` | Phase 1 | 현재 로봇 자세가 zero/model 기준 자세와 불일치 | `CLOSED` | [상세](#ft-20260808-03-zero-기준-자세-불일치) |
| `FT-20260808-04` | 2026-08-08 | `SYNC` | Phase 1 | 설정 1000 Hz와 실제 wrench 갱신 약 500 Hz 불일치 | `MITIGATED` | [상세](#ft-20260808-04-aft-sample-rate-설정과-실제-갱신률-불일치) |
| `FT-20260808-05` | 2026-08-08 | `RUNTIME` | Phase 1 | launch Ctrl-C 시 collector cleanup이 두 번째 SIGINT로 중단 | `CLOSED` | [상세](#ft-20260808-05-ros-launch-종료-중복-sigint) |
| `FT-20260811-01` | 2026-08-11 | `OPS` | Phase 0 | 수집 가이드의 Chrony 상대경로가 현재 작업 디렉터리에서 실패 | `CLOSED` | [상세](#ft-20260811-01-chrony-helper-경로-오류) |

새 실패는 기존 행을 보존하고 `FT-YYYYMMDD-NN` 형식의 새 행으로 추가한다.

## FT-20260811-01: Chrony helper 경로 오류

- 상태: CLOSED
- 분류: OPS
- 누가: free-space 데이터 수집 절차를 수행한 운용자
- 언제: 2026-08-11 11:40 KST, 실제 장비 입력 상태 확인 단계
- 어디서: PC의 `/home/vision/dualarm_ws/src/ft_fb_leaderarm`
- 무엇을: 가이드의 `./scripts/dualarm_chrony_mode.sh status`가
  `No such file or directory`로 실패했다.
- 왜: helper는 `fb_leaderarm/scripts`에 있는데 가이드는 `ft_fb_leaderarm`로 이동한
  뒤 해당 상대경로를 실행하도록 적혀 있었다.
- 어떻게: Chrony 명령은 실행되지 않았지만 ObserverInput과 AFT wrench는 각각
  publisher 1개, 약 1000 Hz로 정상 수신됐다.

### 조치와 영향

- 수집 가이드의 명령을 기존 helper의 절대경로로 수정했다.
- `fb_leaderarm` 코드와 Chrony helper 자체는 수정하지 않았다.
- 영향은 Chrony 상태 확인 명령에만 한정되며 robot, controller, AFT 발행과 수집
  설정은 바뀌지 않는다.
- 절대경로로 재실행하여 SBC source `192.168.112.17`이 `^*`, direct relative bound
  `0.041132 ms <= 1.0 ms`, `valid=true`, 최종 `GO`임을 확인하고 종료했다.

## FT-20260808-01: Fz zero 반복성과 정지 noise

- 상태: OPEN
- 분류: ZERO
- 누가: 운용자, 오른팔 follower와 `/aft_sensor2`
- 언제: 2026-08-08, driver/impedance/AFT ON과 hardware zero-set 후
- 어디서: SBC `/aft_controller_manager`, `/aft_sensor2/wrench`; PC
  `ft_free_space_collector`
- 무엇을: 같은 고정 자세에서 Fz의 tare별 5초 median이
  `-0.06, -0.08, +0.29 N`으로 달랐고, 1초 zero gate가 간헐 실패했다.
- 왜: 관찰된 직접 원인은 Fz std `0.20~0.24 N`과 당시 gate `0.20 N`의 경계
  중첩이다. tare 중심값 차이에는 cable strain, servo 미세운동, warm-up/temperature,
  sensor firmware tare 반복성이 원인 후보이며 아직 분리되지 않았다.
- 어떻게: 고정 초기 자세에서 bias 토픽에 `true`를 3회 독립 발행하고 매회 5초
  wrench 통계를 측정했다. 추가로 같은 tare를 유지한 30초와 collector diagnostics
  11개 창을 측정했다.

### 실행 조건

- git commit 또는 source hash: dirty worktree, 2026-08-08 working copy
- model.ts / metadata SHA-256: 해당 없음, sensor 검증 단계
- zero_set_id: 예비 측정이라 dataset ID 미발급
- payload_id: 미확정
- controller_config_hash: 미확정
- sensor/tool 장착 상태: 오른팔 AFT 장착, 케이블 고정 및 tool/payload 불변 확인
- feedback stage: OFF
- 관련 명령: `ros2 topic pub --once /aft_sensor2/bias_setting std_msgs/msg/Bool "{data: true}"`

### 정량 결과

- 30초 force median/std: `[0.27,0.20,0.03] / [0.127,0.175,0.235] N`
- 30초 force norm p95/p99/max: `0.693/0.841/1.271 N`
- Tare 1 median/std: `[0.05,-0.09,-0.06] / [0.125,0.171,0.218] N`
- Tare 2 median/std: `[0.03,-0.10,-0.08] / [0.129,0.171,0.223] N`
- Tare 3 median/std: `[0.01,-0.12,0.29] / [0.136,0.179,0.232] N`
- Tare 3 추가 5초 median/std: `[-0.02,-0.08,0.21] /
  [0.127,0.174,0.233] N`
- collector zero diagnostics: 11개 1초 창 중 1개만 `zero_verified`
- zero-set2 후 dry-run: 연속 6개 1초 창 중 5개 통과, 1개는 Fz std
  `0.211 N`으로 `zero_force_noise_too_large`
- source age / pairing: latest pair p99/max `0.830/1.833 ms`
- 정지 state-Fz 30초: 최대 q range `0.00533 deg`, 최대 dq `0.000367 rad/s`,
  Fz-dq 최대 절대 상관 `0.0069`, q/dq 선형 `R²=0.071`
- SBC zero-set launch 후 5초 median/std: `[0.15,-0.12,-0.20] /
  [0.124,0.176,0.210] N`, force norm max `1.200 N`
- `aft_zero_set2` 실제 tare 후 5초 median/std: `[0.04,0.05,-0.17] /
  [0.129,0.180,0.248] N`, median norm `0.182 N`, force norm
  p95/p99/max `0.601/0.699/0.993 N`
- 같은 tare의 60초 median/std: `[0.05,0.09,-0.43] /
  [0.125,0.173,0.203] N`, force norm p95/p99/max
  `0.793/0.933/1.327 N`
- 60초의 첫 10초→마지막 10초 median 변화: `[0.00,0.01,0.05] N`;
  선형 drift `[0.011,0.009,0.013] N/min`
- 500 Hz command 후 10초 median/std: `[0.24,0.07,-1.13] /
  [0.121,0.172,0.207] N`; median norm `1.157 N`

### Artifact

- raw NPZ/JSON: `/tmp/aft_sensor2_noise_1786167728.npz` (임시 파일)
- terminal/ROS log: 2026-08-08 현재 작업 세션

### 원인 분석

관찰:

- bias subscriber는 1개이고 bias command도 회차당 한 번만 발행됐다.
- wrench publish는 1,000 Hz이나 연속 force 값의 약 50%가 완전히 같다.
- 같은 tare의 60초 Fz std는 `0.203 N`으로 5초값 `0.248 N`보다 낮았지만
  gate `0.20 N`을 소폭 넘었다. 1초 구간 Fz median spread는 `0.28 N`이었다.
- 공식 e-Manual의 force noise-free resolution(STD)은 `0.4 N`이므로 당시
  `0.20 N` gate는 제조사 STD보다 엄격했다. 현재 gate는 `0.40 N`이다.
- Tare 3의 양의 Fz 중심값은 다음 5초에도 유지되어 단일 sample noise만은 아니다.
- FT-state timing은 현재 latest pairing도 3 ms 제한 안에 충분히 들어온다.
- controller가 보고한 q/dq와 Fz의 선형 상관은 낮아 관절 미세운동을 주원인으로
  보기 어렵다.

미확정 가설:

- joint state에 나타나지 않는 구조 진동이 Fz에 포함된다.
- 케이블 strain/체결력이 tare 시점마다 달라진다.
- warm-up 또는 sensor 내부 temperature compensation의 settling이 부족하다.
- AFT firmware 자체의 bias 반복성이 약 `0.37 N` 범위다.

### 조치

- 수정 내용: 공식 sensor STD에 맞춰 shared verifier, collector와 observer의 축별
  STD gate를 `0.20 → 0.40 N`으로 변경했다. zero-set launch 종료 결함은 별도
  `FT-20260808-02`에서 수정했다.
- 영향 범위: zero 준비/valid 판정만 완화된다. 1 N model gate와 contact threshold는
  변경하지 않았다.
- rollback 방법: `zero_force_axis_std_max_n`과 verifier 기본값을 `0.20 N`으로
  되돌린다. sensor bias는 다음 정상 zero-set으로 갱신한다.

### 재검증

- 동일 조건 재실험 결과: 3회 예비 측정에서 Fz median spread `0.37 N`
- 0.40 N gate 실기 결과: zero-set2 후 연속 7개 1초 창 모두 `zero_verified`,
  최대 축 std `0.205 N` 이하
- 다른 zero-set/time/power-cycle 재실험 결과: 10회, warm-up, power-cycle 시험 대기
- 최종 상태: OPEN; cable/servo/warm-up 원인 분리 전 본 dataset 수집 보류

## FT-20260808-02: SBC zero-set node 종료 교착

- 상태: MITIGATED
- 분류: ZERO
- 누가: SBC `gene2`의 `aft_zero_set_sensor1`, `aft_zero_set_sensor2`
- 언제: 2026-08-08 13:51:05 실행 후 5분 이상
- 어디서: `/home/vision/doosan_ws/src/aft_can_hardware`, ROS domain 7
- 무엇을: 100개 wrench를 평균하고 hardware bias를 요청한 뒤 `auto_exit=true`인
  두 node가 종료되지 않고 남았다.
- 왜: 두 main thread가 `futex_wait_queue`에 있고, 코드가 subscription callback
  안에서 `rclpy.shutdown()`을 직접 호출한다. domain/QoS/topic 연결은 정상이므로
  callback 내부 shutdown 교착이 직접 원인으로 판단된다.
- 어떻게: `ros2 launch aft_can_hardware aft_zero_set.launch.py`를 실행했다.

### 실행 조건

- AFT driver 시작: 2026-08-08 13:13:27
- zero launch 시작: 2026-08-08 13:51:04
- driver/zero 환경: `ROS_DOMAIN_ID=7`, `rmw_cyclonedds_cpp`,
  `ROS_LOCALHOST_ONLY=0`
- 재현 당시 source/install SHA-256: zero script `cc105304...f317a`, launch
  `1c78eecf...6034`; source와 install 일치
- legacy source/install SHA-256: zero script `cc105304...f317a`, launch
  `1c78eecf...6034`; 원본 복구 및 source/install symlink 확인
- 새 source SHA-256: `aft_zero_set2.py` `b29f0d3...8496`; 새 script/launch의
  source/install symlink 확인
- 원본 백업: SBC `/tmp/aft_zero_set_backup_20260808/`
- repository 상태: legacy zero script/launch는 원본을 유지하고, zero-set2
  script/launch/test를 추가했다. 그 외 AFT 파일의 기존 미커밋 수정은 보존했다.

### 관찰

- sensor2 wrench publisher/subscriber는 reliable QoS로 정상 연결됐다.
- `/aft_sensor2/zeroset` publisher는 0개였다.
- 5초간 `/aft_sensor2/bias_setting` 추가 발행은 없었다.
- `AftCanHardware::on_configure()`의 `bias_setting_mode(true)`는 반환 CAN frame을
  전송하지 않고 버리는 무효 호출이다. 실제 tare 경로가 아니므로 원인에서 제외했다.
- launch 전후 Fz 중심값이 `+0.21 N`에서 `-0.20 N`으로 바뀌어 hardware bias
  1회는 전달된 것으로 판단한다.
- 두 node는 반복 tare하지 않지만 종료되지 않아 다음 launch와 중복될 수 있다.
- 100개 평균값은 hardware command에 전달되지 않는다. firmware에는
  `Bool(true)`에 해당하는 bias CAN command만 전달된다.

### 조치

1. 수정 동작은 `aft_zero_set2.py`로 분리했다. legacy `aft_zero_set.py`는 원본을
   유지한다.
2. callback은 종료 flag만 설정하고, main spin loop가 callback 반환 뒤 종료한다.
3. `destroy_node()`와 `rclpy.shutdown()`은 main thread의 `finally`에서 실행한다.
4. bias subscriber가 없거나 DDS ACK가 1초 안에 오지 않으면 성공 종료하지 않는다.
5. launch는 `sensor_name` 인자로 한 센서만 실행하며 기본값은 `aft_sensor2`다.
6. `test/test_aft_zero_set2_shutdown.py`로 bias 1회, callback 후 ROS context 유지,
   종료 flag를 검사한다.

### 재검증

- 격리 `ROS_DOMAIN_ID=97` 회귀 테스트: PASS
- 격리 process 통합 테스트: bias `data=true` 1회,
  `ZERO_PROCESS_RC=0`, `BIAS_ECHO_RC=0`
- `aft_zero_set2.launch.py --show-args`: `sensor_name`, 기본값 `aft_sensor2` 확인
- `/tmp` 격리 colcon build PASS; legacy/new script와 launch 모두 설치 확인
- subscriber가 없는 격리 launch: CAN 요청 없이 오류를 남기고 대기함을 확인
- 실제 domain 7 실행: 오른팔 bias 1회, process clean exit,
  `AFT_ZERO_SET2_RC=0`
- 영향: 기본 launch가 sensor1을 함께 tare하지 않으며, 정상 전달 뒤 node가 종료된다.
- 제한: DDS ACK는 AFT broadcaster 수신까지만 뜻하며 CAN/센서 내부 성공 응답은
  아니다. 실제 wrench 전후값으로 별도 판정한다.
- legacy 영향: `aft_zero_set.launch.py`는 요청에 따라 원본을 유지하므로 sensor1/2를
  함께 실행하고 callback 종료 교착이 남아 있다.
- rollback: zero-set2 파일과 CMake 설치 항목을 제거한다. legacy에는 영향이 없다.
- 최종 상태: MITIGATED; 안전한 단일 sensor 운용은 `aft_zero_set2`를 사용한다.

## FT-20260808-03: zero 기준 자세 불일치

- 상태: CLOSED
- 분류: OPS
- 누가: 오른팔 follower, `/TorqueRtR`, zero-set 운용자
- 언제: 2026-08-08 zero-set 수정 후 실제 hardware 재검증 직전
- 어디서: `/contact_state/observer_input`, `config/collector.yaml`, `config/observer.yaml`
- 무엇을: 설정 기준은 `[5.5, 52, 112, 28, -107, -35] deg`지만 현재 정지 자세는
  약 `[5.31, -51.67, -112.20, -27.73, -106.42, -35.12] deg`였다.
- 왜: J2~J4 부호가 달라 최대 오차가 J3 약 `224.2 deg`이며 1 deg gate를 넘는다.
  이 자세에서 zero하면 이후 수집 시작 자세와 같은 조건이라는 보장이 없다.
- 어떻게: 실제 publisher가 `/TorqueRtR` 하나임을 확인하고 C++ 소스를 추적했다.
  Doosan RT API의 `actual_joint_position`을 그대로 degree→radian 변환하므로 메시지
  부호 변환 버그는 아니다.

### 조치와 다음 단계

- 실제 hardware zero-set은 보류했다. robot/collector 설정은 변경하지 않았다.
- 영향: 당시 collector는 `not_at_fixed_zero_pose`로 zero gate를 통과할 수 없었다.
- 다음: 안전한 robot 조작 절차로 설정 기준 자세에 복귀한 후 실제 zero-set을
  재검증한다. 의도한 기준 자세가 현재 음수 자세라면 기존 모델·데이터 계약 전체를
  함께 변경해야 하므로 별도 결정한다.
- 재검증: 약 `[5.47,51.92,112.11,27.99,-106.91,-35.01] deg`, 모든
  `dq≈0`, 최대 자세 오차 약 `0.19 deg`를 확인한 뒤 실제 zero-set2를 수행했다.
- 재발: 500 Hz/gate 변경 후 tare 전 확인에서 약
  `[5.47,-51.88,-112.20,-27.99,-106.89,-35.01] deg`, 모든 `dq≈0`,
  `has_active_command=false`였다. 기준 자세 복귀 전 zero-set과 episode를 보류했다.
- 재검증: 이후 3초 연속 3,001개 state가 약
  `[5.47,51.88,112.14,27.99,-106.89,-35.01] deg`, active command 0,
  최대 dq `0.0 rad/s`임을 확인하고 zero-set2를 수행했다.
- 최종 상태: CLOSED

## FT-20260808-04: AFT sample-rate 설정과 실제 갱신률 불일치

- 상태: MITIGATED
- 분류: SYNC
- 누가: SBC `AftCanHardware`, `AftCanBroadcaster`, 오른팔 `aft_sensor2`
- 언제: 2026-08-08 zero-set2 후 60초 정지 측정과 CAN 확인 중
- 어디서: SBC `can1`, `/aft_sensor2/wrench`; PC collector 입력
- 무엇을: 변경 전 YAML은 sensor/controller를 1000 Hz로 설정했지만 ROS wrench의 연속
  force 값 약 50%가 중복되고, 실제 CAN force/torque 쌍은 약 2 ms 주기였다.
- 왜: `AftCanHardware::on_configure()`가 `sample_rate_setting_mode(configured_rate)`의 반환
  CAN frame을 전송하지 않아 설정값이 센서에 적용되지 않는다. broadcaster는
  1000 Hz controller update마다 마지막 state를 발행한다.
- 어떻게: 60초 ROS stamp/값 통계, `candump -L -n 20 can1`, 설정·초기화·runtime
  command 경로를 대조했다.

### 정량 결과와 영향

- ROS publish/stamp rate: `1000.07 Hz`, 최대 stamp gap `1.066 ms`
- 연속 force 중복률: `50.0008%`, 값이 바뀐 유효 rate 약 `499.99 Hz`
- raw CAN: force ID `0x001`과 torque ID `0x002`가 한 쌍으로 오고, 쌍 간격 약 2 ms
- CAN 상태: 1 Mbps, `ERROR-ACTIVE`, bus error/drop/missed/bus-off 0
- 공식 e-Manual: 최대 1000 Hz, 변경 가능 범위 100~1000 Hz이며 1000 Hz command
  parameter는 `0x03E8`, 500 Hz는 `0x07D0`이다.
- collector는 stamp 기반 `RateGate(262.5 Hz)`를 사용하므로 목표 저장률보다 실제
  갱신률이 높다. 즉시 수집 불능은 아니지만, 중복 publish stamp는 실제 sensor
  acquisition 시각이 아니므로 동적 구간의 시간 정렬 오차 요인이다.
- 이 불일치는 정지 Fz noise가 sensor rate 때문에 발생했다는 증거는 아니다.
- 코드상 1 ms `read()`마다 CAN frame 하나만 소비하고 sensor sample 하나는
  force/torque 두 frame이다. 따라서 sensor를 1000 Hz로 바꾸면 2000 frame/s 입력을
  1000 read/s로 처리해 backlog가 생길 수 있다는 것은 소스 기반 추론이다.

### 조치와 재검증

- 수정 내용: sensor2 YAML을 `500 Hz`로 바꾸고 실제 500 Hz를 임시 운용 계약으로
  기록했다. controller update 1000 Hz와 sensor1은 유지했다.
- SBC 설정 SHA-256: 변경 전 `7ac48d4e...f8e49`, 변경 후
  `5cfa2eb8...cebbb`; 백업 `/tmp/aft_sensor_settings_7ac48d4e_before_500.yaml`
- 이유: 현재 262.5 Hz 목표에는 실측 500 Hz가 충분하고, 1000 Hz는 driver read
  처리량을 먼저 수정해야 안전하게 검증할 수 있다.
- 영향 범위: 문서와 Phase 1/3의 검증 항목만 변경된다.
- 다음: 1000 Hz가 실제로 필요할 때만 driver `read()`가 한 cycle에 대기 frame을
  모두 drain하도록 먼저 수정한다. 그 뒤 configure rate command 전송과 CAN 주기,
  backlog, ROS 중복률, noise를 함께 재검증한다.
- 공식 근거: [AIDIN AFT200-D80-C e-Manual](https://emanual.oopy.io/aft-200-d80-c-eng)
- 통합 기록: [AFT sensor 사양과 현재 이슈](AFT_sensor_issue.md)
- rollback: 문서 변경만 되돌린다.
- 최종 상태: MITIGATED; 현재는 500 Hz로 운용하고 1000 Hz driver 지원은 보류

### 500 Hz runtime 재검증

- `/aft_sensor2/sample_rate_setting` subscriber 1개 확인 후 `Int32(data=500)` 1회 발행
- 발행 후 ROS stamp rate `1000.07 Hz`, 연속 force 중복률 `50%`, 유효 갱신 약 500 Hz
- 별도 `candump` socket에서는 outgoing `0x102` frame을 직접 보지 못했으므로 CAN
  ACK 성공으로 과장하지 않는다. 관찰된 data rate가 500 Hz 계약을 유지한 것만 확정한다.

### collector smoke 재검증

- `/tmp` 정지 episode: `16.286 s`, 4,276 sample, `262.495 Hz`
- 최대 gap `4.030 ms`, sync p99/max `1.223/1.397 ms`, rejection 0
- collector `training_accepted=true`; validator는 단일 zero-set group이라 정상 NO-GO

## FT-20260808-05: ROS launch 종료 중복 SIGINT

- 상태: CLOSED
- 분류: RUNTIME
- 누가: PC `ft_free_space_collector`; 같은 main 패턴의 `ft_contact_observer`
- 언제: 2026-08-08 collector dry-run을 Ctrl-C로 종료할 때
- 어디서: `collect_free_space.launch.py`, 두 Python node의 `main()` cleanup
- 무엇을: collector는 종료됐지만 `destroy_node()`에서 `KeyboardInterrupt`
  traceback을 남기고 child exit code `-2`가 됐다.
- 왜: terminal SIGINT 뒤 ROS launch가 child에 SIGINT를 다시 전달하고, 기존 코드는
  cleanup 중 추가 SIGINT를 막지 않았다.
- 어떻게: episode를 시작하지 않은 collector를 launch하고 Ctrl-C로 종료했다.

### 조치와 영향

- cleanup 동안만 SIGINT를 무시하고 완료 후 기존 handler를 복구하도록 collector와
  observer를 수정했다. SIGTERM 동작과 실행 중 첫 SIGINT는 바꾸지 않았다.
- collector가 수집 중이면 기존 `destroy_node()`의 episode 저장을 끝낼 수 있다.
- wrench 처리, zero gate, 모델 추론, robot/AFT node에는 영향이 없다.
- 회귀 테스트는 cleanup 중 두 번째 SIGINT, node destroy, ROS shutdown, handler
  복구를 두 node에서 검사한다.
- rollback: 두 `main()`의 signal guard와 `test_ros_shutdown.py`를 제거한다.
- 재검증: syntax/diff check와 회귀 테스트 `2 passed`; 같은 실제 launch를 Ctrl-C로
  종료해 traceback 없이 `process has finished cleanly`, launch exit code 0 확인
- 최종 상태: CLOSED

## 상세 기록 템플릿

```markdown
## FT-YYYYMMDD-NN: 짧은 제목

- 상태: OPEN
- 분류: SENSOR/ZERO/SYNC/DATA/MODEL/RUNTIME/CONTACT/FEEDBACK/ROBOT/OPS
- 누가: 운용자, 사용한 로봇/팔
- 언제: 날짜·시간, 전원 ON 후 경과 시간
- 어디서: SBC/PC, node, topic, 파일 경로
- 무엇을: 기대값과 실제 증상
- 왜: 현재 원인 가설과 근거
- 어떻게: 재현 순서

### 실행 조건

- git commit 또는 source hash:
- model.ts / metadata SHA-256:
- zero_set_id:
- payload_id:
- controller_config_hash:
- sensor/tool 장착 상태:
- feedback stage: OFF/40%/100%
- 관련 명령:

### 정량 결과

- force max/p95/RMSE:
- false contact activation:
- contact miss:
- inference p99/max:
- source age / record gap:
- max feedback torque:
- max pose step:
- velocity reversal rate:

### Artifact

- raw NPZ/JSON:
- Leader CSV:
- analyzer report:
- authorization:
- terminal/ROS log:

### 원인 분석

관찰과 추측을 분리해서 작성한다.

### 조치

- 수정 내용:
- 영향 범위:
- rollback 방법:

### 재검증

- 동일 조건 재실험 결과:
- 다른 zero-set/time/power-cycle 재실험 결과:
- 최종 상태:
```

## 기록 규칙

1. raw artifact는 수정하지 않는다.
2. 실패한 모델과 report도 삭제하지 않는다.
3. threshold를 바꿔 통과시킨 경우 기존 threshold 실패를 별도 기록한다.
4. contact가 섞인 free-space episode는 수정하여 재사용하지 않고 폐기 표시한다.
5. 원인이 여러 개면 대표 ID 아래에 가설별 증거를 나눈다.
