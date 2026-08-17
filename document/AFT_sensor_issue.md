# AFT sensor 사양과 현재 이슈

## 목적

오른팔 `AFT200-D80-C`의 공식 사양, 실제 측정률, zero gate와 남은 문제를 한 곳에서
관리한다. 측정값과 소스에서 확인한 사실은 추측과 분리한다.

## 공식 사양

2026-08-08 확인 기준
[AIDIN 공식 AFT200-D80-C e-Manual](https://emanual.oopy.io/aft-200-d80-c-eng)의
주요 사양은 다음과 같다.

| 항목 | 공식 값 |
|---|---:|
| force resolution | `0.15 N` |
| force noise-free resolution (STD) | `0.4 N` |
| torque noise-free resolution (STD) | `0.025 Nm` |
| 최대 sample rate | `1000 Hz` |
| 변경 가능한 output rate | `100~1000 Hz` |
| 권장 warm-up | 사용 전 최소 10분, 안정화 약 30분 |

공식 문서는 움직임 중 cable이 당겨지면 output noise가 발생할 수 있다고 경고하고
M5 체결 토크 `5.2 Nm`도 명시한다.

## 현재 rate 계약

| 구간 | 현재 값 | 의미 |
|---|---:|---|
| AFT sensor2 sample rate | `500 Hz` | force/torque 한 쌍이 약 2 ms마다 생성 |
| CAN frame 유입 | `1000 frame/s` | sample당 force와 torque 두 frame |
| AFT controller update | `1000 Hz` | cycle당 CAN frame 하나를 읽기 위해 유지 |
| ROS wrench publish | 약 `1000 Hz` | controller update마다 마지막 state 발행 |
| 새 force 값 갱신 | 약 `500 Hz` | ROS 연속 force 값 약 50% 중복 |
| collector/runtime | `262.5 Hz` | 모델 학습·추론 계약 |

세 rate는 다음처럼 서로 다른 의미다.

```text
AFT sensor (500 Hz):  2 ms마다 force frame + torque frame 한 쌍 생성
driver read (1000 Hz): 1 ms마다 CAN frame 하나를 읽음
ROS publish (1000 Hz): 1 ms마다 현재 cache의 6축 wrench를 발행
collector (262.5 Hz):  약 3.81 ms마다 학습용 row를 선택
```

따라서 ROS의 완전한 6축 메시지가 정확히 두 번 동일하다는 뜻은 아니다. 한 cycle은
새 force와 이전 torque, 다음 cycle은 같은 force와 새 torque를 포함할 수 있다.
force 세 축만 보면 약 50%가 반복되고, 새 force/torque 한 쌍의 유효 주기는 약
500 Hz다.

sensor2 YAML은 `500 Hz`로 맞췄지만 현재 driver `on_configure()`는 rate command
frame을 생성만 하고 CAN으로 보내지 않는다. 따라서 AFT 시작 후 아래 runtime
command로 500 Hz를 명시하고 CAN 주기를 확인한다.

```bash
ros2 topic pub --once /aft_sensor2/sample_rate_setting \
  std_msgs/msg/Int32 "{data: 500}"
```

이 명령은 wrench를 500회 발행하는 명령이 아니다. `Int32(data=500)` 설정 메시지
하나를 보내 sensor sample period를 2 ms로 지정하는 one-shot command다. ROS wrench
publish rate는 controller 설정 때문에 계속 약 1000 Hz다.

`aft_controller_manager.update_rate`를 500 Hz로 낮추면 안 된다. 현재 driver는 한
cycle에 CAN frame 하나만 읽으므로 controller를 500 Hz로 낮추면 sensor의
1000 frame/s를 절반밖에 소비하지 못해 backlog가 생긴다.

## 문제 분석: 육하원칙

| 구분 | 내용 |
|---|---|
| 누가 | 오른팔 `aft_sensor2`, SBC `AftCanHardware/AftCanBroadcaster`, PC collector/observer |
| 언제 | AFT warm-up과 hardware zero-set 후 정지·동적 free-space 측정 시 |
| 어디서 | SBC `can1`, `/aft_sensor2/wrench`; PC zero gate와 model target |
| 무엇을 | Fz std가 기존 `0.20 N` gate를 간헐 초과하고 ROS 1000 Hz의 약 50%가 중복됨 |
| 어떻게 | 5/30/60초 통계, 1초 gate 창, raw CAN 주기와 driver 소스를 대조함 |
| 왜 | 기존 gate가 공식 force STD `0.4 N`보다 엄격했고 sensor 500 Hz와 controller publish 1000 Hz를 같은 rate로 해석했기 때문 |

## 확인된 측정값

- zero-set2 후 60초 force std: `[0.125,0.173,0.203] N`
- 60초 force norm p95/p99/max: `0.793/0.933/1.327 N`
- median 제거 force-vector p99/max: `0.628/0.973 N`
- 연속 force 중복률: `50.0008%`, 유효 force 갱신률 약 `499.99 Hz`
- 1초 dry-run: 기존 0.20 N gate에서 6개 창 중 5개 통과, 1개 Fz std
  `0.211 N`으로 실패
- CAN 1 Mbps 상태: bus error/drop/missed/bus-off 0
- 500 Hz runtime command 후 10초: ROS `1000.07 Hz`, force 중복률 `50%`,
  force std `[0.121,0.172,0.207] N`
- 같은 10초 median은 `[0.24,0.07,-1.13] N`, norm `1.157 N`으로 STD gate는
  만족하지만 별도 median norm `1.0 N` gate는 실패했다.
- 기준 자세에서 zero-set2를 다시 수행한 뒤 7개 연속 1초 창이 모두
  `zero_verified`였고 최대 축 std는 `0.205 N` 이하였다.
- 2026-08-11 AFT 재시작 후 `/aft_sensor2/sample_rate_setting` subscriber 1개와
  `Int32(data=500)` 1회 발행을 다시 확인했다.

## 적용한 변경과 영향

1. SBC `aft_sensor2.sample_rate`를 `1000 → 500 Hz`로 변경했다.
2. collector/observer/shared verifier의 force 축 STD gate를 `0.20 → 0.40 N`으로
   변경했다.
3. sensor1과 AFT controller `update_rate=1000`은 변경하지 않았다.

gate 완화로 공식 센서 noise 범위의 정지 데이터는 zero 검증을 통과할 수 있다.
반면 허용 noise가 커지므로 gate 통과만으로 모델의 최대 force-vector 오차 `1 N`이
보장되지는 않는다. 모델 validation/test의 1 N gate와 contact threshold는 그대로다.

sensor rate 설정 변경은 현재 실측 500 Hz와 YAML을 일치시키므로 정상 상태의 실제
data rate는 달라지지 않는다. 달라지는 것은 재현 가능한 설정 계약과 runtime command
값이다. ROS wrench publish는 계속 약 1000 Hz이며 중복도 남는다.

## STD gate와 median-offset gate의 차이

fixed zero pose에서 최근 1초 force를 검사할 때 두 gate를 모두 통과해야 한다.

1. 축별 STD gate `≤ 0.40 N`: 값이 중심 주변에서 얼마나 흔들리는지 검사한다.
2. median norm gate `≤ 1.0 N`: 1초 중심값이 zero에서 얼마나 벗어났는지 검사한다.

```text
예 A: median=[0.20,-0.10,0.30] N
      norm=sqrt(0.20²+0.10²+0.30²)=0.374 N → offset gate 통과

예 B: median=[0.24,0.07,-1.13] N
      norm=sqrt(0.24²+0.07²+1.13²)=1.157 N → offset gate 실패
      reason=zero_force_offset_too_large
```

예 B는 STD가 `0.207 N`으로 작아도 “조용하지만 중심이 zero에서 벗어난 상태”라서
gate를 통과하지 못한다.

여기서 **collector가 차단하는 것은 새 episode의 수집 시작 한 가지뿐**이다.

- 차단함: `/ft_free_space_collector/start_episode` 요청을 `success=false`로 거부
- 차단하지 않음: `aft_zero_set2`, `/aft_sensor2/wrench` publish, robot/controller,
  이미 시작된 episode
- observer의 별도 동작: topic publish는 계속하지만 zero 검증 전 observation을
  `valid=false`로 표시

collector node는 종료되지 않으며 diagnostics도 계속 발행한다. 사용자는 start
service 응답의 `message`와 diagnostics의 `zero.ready`, `zero.reason`으로 차단을
확인한다. `ft_fb_leaderarm` 전용 GUI를 사용하면 같은 상태가 Zero Gate 배지와
START 실패 팝업에 표시된다.

```text
response:
std_srvs.srv.Trigger_Response(success=False,
  message='fixed-pose zero verification failed: zero_force_offset_too_large')

diagnostics 핵심:
data: '{"collecting": false, "zero": {"ready": false,
  "reason": "zero_force_offset_too_large", ...}, ...}'
```

여기서 tare 중심 이동은 같은 fixed zero pose로 돌아왔는데도 zero-set 직후보다
중심값이 달라지는 현상을 뜻한다. 온도, cable strain, 체결 응력 또는 sensor 내부
변화가 후보이며 아직 하나로 확정하지 않았다. 다른 로봇 자세에서 중력 때문에
wrench가 달라지는 것은 정상 free-space wrench이므로 tare drift로 판정하지 않는다.

## 남은 문제와 다음 검증

- runtime 500 Hz command 후 raw CAN pair 주기가 계속 약 2 ms인지 확인한다.
- DDS command는 subscriber 1개에 발행됐고 후속 유효 rate 500 Hz를 확인했다.
  별도 `candump` socket에서 송신 `0x102` frame은 직접 관찰되지 않았으므로 hardware
  ACK를 확인한 것으로 기록하지 않는다.
- 새 0.4 N gate로 collector/observer의 연속 zero 창을 재검증한다.
- 첫 동적 episode에서 sync error, record rate와 wrench 중복 영향을 측정한다.
- 정지 데이터의 4 ms causal 평균은 force-vector max를 `0.973 → 0.797 N`으로
  낮췄지만 contact latency가 확인되지 않아 아직 구현하지 않는다.
- 실제 1000 Hz sensor sample이 필요할 때만 driver가 cycle마다 대기 CAN frame을
  모두 drain하도록 수정한 뒤 rate command 자동 적용을 구현한다.

상세 실험 이력은 [failure log](failure_log.md)와
[FT sensor 확인 목록](FTsensor_check_list.md)에 보존한다.

## 2026-08-08 collector smoke 결과

정식 dataset과 분리한 `/tmp` 정지 episode로 저장 경로를 검증했다.

| 항목 | 결과 |
|---|---:|
| duration / samples | `16.286 s / 4276` |
| 실제 저장률 | `262.495 Hz` |
| 최대 record gap | `4.030 ms` |
| sync error p99 / max | `1.223 / 1.397 ms` |
| sync / invalid / duplicate-state rejection | `0 / 0 / 0` |
| collector 판정 | `training_accepted=true` |

artifact는 `/tmp/ft_fb_collector_gate_dryrun/right_free_space_20260808_151919.npz`다.
metadata의 payload/controller는 `unconfirmed_smoke`이므로 정식 학습에 사용하지
않는다. dataset validator는 독립 zero-set group이 하나뿐이라 최소 3개 계약으로
NO-GO했으며 이는 정상 안전 차단이다.
