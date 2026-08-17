# FT sensor 확인 목록

## 원칙

1 N은 모델 오차만의 예산이 아니라 sensor noise, drift, timestamp mismatch,
frame 오차를 모두 포함한 결과 예산이다. sensor 관련 오차가 전체 1 N에
근접하면 어떤 모델도 요구 성능을 안정적으로 만족하기 어렵다.

동일 zero 자세에서 zero 직후 값이 0에 가까운 것은 반복성 증거가 아니다.
매 zero-set 후 여러 검증 자세로 이동한 측정값을 비교한다.

## 체크리스트

| 완료 | 항목 | 방법 | 기록값/판정 |
|---|---|---|---|
| [ ] | warm-up drift | ON 후 0/15/30/60/120분 같은 자세 기록 | session 동안 force 변화, 온도 |
| [ ] | zero 반복성 | 같은 조건에서 zero 10회, 여러 자세 왕복 | group 간 force-vector 분산/최대차 |
| [ ] | power-cycle 반복성 | SBC/AFT 재부팅 전후 반복 | 재부팅 전후 offset 변화 |
| [ ] | 단기 noise | 정지 상태 30~60초 | 축별 std, force norm p95/max |
| [ ] | 장시간 bias | 재-zero 없이 예상 작업 시간 기록 | 시간에 따른 drift slope/max |
| [ ] | 온도 영향 | sensor/주변 온도와 wrench 동시 기록 | 온도-bias 상관 |
| [ ] | 자세 반복성 | 같은 자세를 양 방향에서 접근 | hysteresis |
| [ ] | payload/tool | 체결과 질량 중심 확인 | `payload_id`, 체결 토크, 사진 |
| [ ] | cable strain | 모든 자세에서 cable 간섭 확인 | cable 방향별 offset |
| [ ] | frame/부호 | 알려진 +X/+Y/+Z 힘 적용 | sensor/base 축과 부호 |
| [ ] | lever arm | sensor→TCP translation 측정 | torque 변환 오차 |
| [ ] | timestamp sync | FT와 observer input stamp 비교 | max/p95 sync error |
| [ ] | 발행률/jitter | `ros2 topic hz`, timestamp diff | rate, max gap, drop count |
| [ ] | saturation | 허용 범위 내 하중 후 제거 | rail 여부, offset 복귀 |
| [ ] | overload recovery | 안전 범위 시험 후 정지 기록 | 영점 복귀 시간 |
| [ ] | EMI/CAN | 모터/전원 변화 중 기록 | spike, reconnect, packet loss |
| [ ] | free false contact | 다양한 무접촉 움직임 | activation 0회 |
| [ ] | controlled contact | 제한한 방향/크기 접촉 | 검출/해제/방향/latency |

## 현재 software zero gate

collector와 observer는 다음을 만족하기 전에는 valid inference/수집을 시작하지
않는다.

| 항목 | 현재 값 |
|---|---:|
| 초기 joint 자세 | `[5.5, 52.0, 112.0, 28.0, -107.0, -35.0] deg` |
| joint 자세 허용오차 | 1.0 deg |
| 최대 joint 속도 | 0.02 rad/s |
| 안정 시간 | 1.0 s |
| zero force median norm | ≤ 1.0 N |
| 각 force 축 표준편차 | ≤ 0.40 N |

이 gate는 장시간 drift나 독립 zero-set 반복성을 보장하지 않는다.

## 2026-08-08 오른팔 AFT 예비 실측

고정 초기 자세, impedance controller ON, 무접촉 상태에서 측정했다. hardware
zero-set은 `/aft_sensor2/bias_setting`에 `Bool(data=true)`를 한 번 발행하고 각
회차 뒤 5초를 측정했다.

[AIDIN 공식 AFT200-D80-C e-Manual](https://emanual.oopy.io/aft-200-d80-c-eng)은
force resolution `0.15 N`, force noise-free resolution(STD) `0.4 N`, 최대
sample rate `1000 Hz`를 명시한다. 사용 전 최소 10분, 신호 안정화를 위해 약 30분
운전을 권장하며 cable이 움직임 중 당겨지면 noise가 발생할 수 있다고 경고한다.

| 항목 | 결과 |
|---|---|
| 30초 force median | `[0.27, 0.20, 0.03] N` |
| 30초 force std | `[0.127, 0.175, 0.235] N` |
| 30초 force norm p95/p99/max | `0.693 / 0.841 / 1.271 N` |
| Tare 1 median/std | `[0.05,-0.09,-0.06] / [0.125,0.171,0.218] N` |
| Tare 2 median/std | `[0.03,-0.10,-0.08] / [0.129,0.171,0.223] N` |
| Tare 3 median/std | `[0.01,-0.12,0.29] / [0.136,0.179,0.232] N` |
| 세 tare의 Fz median 범위 | `-0.08 ~ 0.29 N`, spread `0.37 N` |
| ROS publish / 연속 force 중복률 | `1,000 Hz / 약 50%` |
| zero-set2 후 60초 median/std | `[0.05,0.09,-0.43] / [0.125,0.173,0.203] N` |
| 60초 force norm p95/p99/max | `0.793 / 0.933 / 1.327 N` |
| 60초 첫→마지막 10초 median 변화 | `[0.00,0.01,0.05] N` |
| 유효 force 갱신률 / raw CAN pair 주기 | `약 500 Hz / 약 2 ms` |
| AFT driver 시작/조회 시 경과 | `13:13:27 / 40분 8초` |
| Chrony direct relative bound | `0.041183 ms`, `valid=true`, `GO` |

관찰상 같은 tare 안에서도 Fz의 1초 median과 std가 변하고, 세 번째 tare의
`+0.29 N` 중심값은 추가 zero-set 없이 다음 5초에도 `+0.21 N`으로 남았다.
따라서 단일 sample 착시뿐 아니라 tare 간 중심값 차이도 함께 존재한다. 현재
당시 `zero_force_axis_std_max_n=0.20`은 실측 Fz noise와 경계가 겹쳐 collector
진단 11개 창 중 1개만 통과했다. 2026-08-08 공식 sensor STD에 맞춰 현재 gate는
`0.40 N`으로 변경했다.

같은 고정 자세에서 30초간 state와 Fz의 상관을 추가 측정했다. 관절 위치
peak-to-peak는 최대 `0.00533 deg`, 관절 속도 최대값은 `0.000367 rad/s`,
Fz-dq 상관계수 절댓값은 최대 `0.0069`, q/dq 선형 설명력은 `R²=0.071`이었다.
따라서 controller가 보고한 관절 미세운동은 현재 Fz 변동의 주원인으로 보기
어렵다. joint state에 나타나지 않는 구조 진동은 이 결과만으로 배제하지 않는다.

운용자가 AFT 케이블 고정과 tool/payload 불변을 확인했다. 실제 SBC 배포 경로는
`/home/vision/doosan_ws`다. 기존 `aft_zero_set.py`와 launch는 원본을 유지하고,
수정 동작을 `aft_zero_set2.py`와 `aft_zero_set2.launch.py`로 분리했다. zero-set2는
callback 종료 교착을 제거하고 `sensor_name`으로 한 센서만 선택한다. 격리 회귀
테스트와 실제 오른팔 tare/clean exit가 통과했으며 기본 센서는 `aft_sensor2`다. DDS
ACK는 broadcaster가 메시지를 받은 것까지만 보장하므로 실제 CAN/센서 성공은
zero 전후 wrench로 판정한다. legacy launch에는 sensor1/2 동시 실행과 종료 교착이
그대로 남아 있으므로 안전한 단일 센서 운용에는 zero-set2를 사용한다.
[FT-20260808-02](failure_log.md#ft-20260808-02-sbc-zero-set-node-종료-교착)는
legacy 보존으로 인해 `MITIGATED`다.

AFT driver의 `on_configure()`에도 `bias_setting_mode(true)` 호출이 있지만, 반환된
CAN frame을 전송하지 않고 버리므로 실제 tare 경로가 아니다. 실제 hardware tare는
`/aft_sensor2/bias_setting` callback이 command buffer를 설정하고 `write()`가 CAN
frame을 전송할 때만 발생한다. 따라서 driver ON 자체는 명시적 zero-set을 대신하지
않는다.

실기 재검증 직전 `/contact_state/observer_input`의 자세는 약
`[5.31,-51.67,-112.20,-27.73,-106.42,-35.12] deg`로, 설정 기준
`[5.5,52,112,28,-107,-35] deg`와 J2~J4 부호가 달랐다. publisher 소스는 Doosan
실제 관절각을 부호 변환 없이 radian으로 바꾸므로 메시지 변환 문제는 아니다.
이 상태에서는 hardware zero와 dataset 수집을 진행하지 않았다. 이후
`[5.47,51.92,112.11,27.99,-106.91,-35.01] deg`, 모든 `dq≈0`으로 복귀해
zero-set2를 수행했으므로 자세 이슈는
[FT-20260808-03](failure_log.md#ft-20260808-03-zero-기준-자세-불일치)에서
`CLOSED` 처리했다.

zero-set2 후 5초간 5,001 sample을 측정한 결과 force median/std는
`[0.04,0.05,-0.17] / [0.129,0.180,0.248] N`, median norm은 `0.182 N`,
force norm p95/p99/max는 `0.601/0.699/0.993 N`이었다. 중심값과 norm max는
median과 norm은 기준 안이었지만 당시 Fz std `0.248 N`이 기존 `0.20 N` 기준을
넘었다. 현재 `0.40 N` gate에서는 이 구간이 통과 대상이며 반복 tare 원인 분리는
별도로 계속한다.

같은 tare를 유지한 60초 측정에서는 Fz std가 `0.203 N`으로 낮아졌지만 당시
`0.20 N` gate를 소폭 넘었고, 1초 구간 Fz median은 `0.28 N` 범위에서 움직였다.
첫 10초와 마지막 10초의 Fz median 차이는 `0.05 N`이라 이 60초 결과만으로 큰
단조 drift를 원인으로 보기는 어렵다. 임시 raw artifact는
`/tmp/aft_sensor2_noise_1786167728.npz`다.

`candump`에서는 `can1`의 force `0x001`/torque `0x002` 쌍이 약 2 ms마다 왔다.
ROS는 1000 Hz로 publish하지만 값이 바뀌는 rate는 약 500 Hz다. sensor2 YAML은
`sample_rate: 500`으로 맞췄지만 driver `on_configure()`에서 command frame만 생성되고
CAN으로 전송되지 않는다. 현재 driver는 controller cycle마다 CAN frame 하나만 읽고 sensor
sample은 force/torque 두 frame이므로 500 Hz를 임시 운용 계약으로 둔다. collector의
262.5 Hz 목표에는 500 Hz가 충분하지만 stamp는 실제 acquisition 시각이 아니므로
동적 데이터의 sync 검증이 필요하다. 자세한 육하원칙과
후속 결정은 [FT-20260808-04](failure_log.md#ft-20260808-04-aft-sample-rate-설정과-실제-갱신률-불일치)에 기록했다.
현재 사양과 운용 계약 요약은 [AFT sensor 이슈](AFT_sensor_issue.md)에 기록한다.

collector dry-run의 연속 6개 1초 zero 창은 기존 gate에서 5개가 통과했고 1개가
Fz std `0.211 N`으로 실패했다. 현재 `0.40 N` gate 재검증 결과는 별도 기록한다.

60초 데이터에서 median을 제거한 force-vector 최대는 raw 500 Hz sample 기준
`0.973 N`이었다. causal 평균 후보는 다음과 같다. 이는 정지 noise 결과이며 접촉
응답이나 움직임 중 residual 성능을 뜻하지 않는다.

| causal 평균 | Fz std | median 제거 force norm p99/max |
|---:|---:|---:|
| 없음 | `0.203 N` | `0.628 / 0.973 N` |
| 2 sample, 약 4 ms | `0.175 N` | `0.522 / 0.797 N` |
| 4 sample, 약 8 ms | `0.143 N` | `0.424 / 0.672 N` |
| 8 sample, 약 16 ms | `0.118 N` | `0.352 / 0.545 N` |

가장 작은 4 ms 후보도 정지 여유는 만들지만 contact latency에 영향을 줄 수 있다.
첫 free-space/contact 검증 데이터에서 raw와 4 ms causal 후보를 오프라인 비교하기
전에는 collector/runtime에 필터를 추가하지 않는다.

driver 시작 시각은 sensor 연속 전송을 시작한 실측 가능한 기준이다. 실제 전기
전원 인가는 이보다 빠를 수 있으므로 정확한 power-on 시각을 대신하지는 않는다.

아직 3회 예비 측정이므로 10회 반복성 시험 완료로 보지 않는다. 다음 순서로
원인을 분리한다.

1. 동일 자세에서 케이블을 고정하고 strain 방향/체결 상태를 기록한다.
2. 별도 accelerometer가 있으면 joint state에 안 잡히는 구조 진동을 확인한다.
3. 가능하면 모터/impedance 비활성 안전 상태와 활성 상태의 정지 noise를 비교한다.
4. 전원 ON 경과시간을 함께 기록하며 zero-set 10회를 완료한다.
5. 원인이 제거된 뒤 1 N error budget을 기준으로 zero gate를 다시 정한다.
6. 첫 움직임/contact 데이터에서 raw와 4 ms causal 평균의 오차·검출 지연을 비교한다.

gate는 공식 STD에 맞춰 `0.40 N`으로 변경했지만 tare 반복성은 별개이므로
[FT-20260808-01](failure_log.md#ft-20260808-01-fz-zero-반복성과-정지-noise)을
`OPEN`으로 유지한다.

500 Hz runtime command 후 10초 측정은 ROS `1000.07 Hz`, force 중복률 `50%`로
유효 500 Hz 계약을 유지했다. force std `[0.121,0.172,0.207] N`은 새 0.40 N
STD gate 안이지만 median `[0.24,0.07,-1.13] N`의 norm이 `1.157 N`이라 median
norm gate는 실패한다. 새 tare 전 pose를 확인했을 때 J2~J4 부호가 기준과 다시
반대여서 tare와 episode를 보류했다.

이후 3초 연속 3,001개 state가 기준 자세 안이고 active command 0, 최대 dq
`0.0 rad/s`임을 확인해 zero-set2를 다시 수행했다. 새 0.40 N gate의 연속 7개
1초 창은 모두 `zero_verified`였고 최대 축 std는 `0.205 N` 이하였다. `/tmp` 정지
smoke episode도 `262.495 Hz`, 최대 gap `4.030 ms`, sync p99/max
`1.223/1.397 ms`, rejection 0으로 저장됐다.

## ObserverInput-AFT 시간 정렬 검증과 향후 개선

### 현재 계약과 한계

- SBC의 `/contact_state/observer_input`과 `/aft_sensor2/wrench`는 약 1,000 Hz로
  발행된다. 현재 AFT driver에서는 force 값이 두 publish마다 갱신되어 연속 force
  중복률이 약 50%이고, collector/runtime이 262.5 Hz로 선택해 사용한다.
- collector는 FT callback 시점의 최신 ObserverInput 하나를 사용한다.
- runtime observer도 timer 실행 시점의 최신 ObserverInput과 최신 FT 하나를
  사용한다.
- 두 source의 도착 시각이 아니라 `header.stamp`를 비교한다.
- `max_sync_error_ms=3.0`, `max_source_age_ms=20.0`을 넘는 조합은 사용하지
  않는다. runtime observer는 미래 stamp도 `clock_future_tolerance_ms=2.0`
  이내에서만 허용한다.
- 잘못 정렬된 조합은 collector에서 `sync_rejections`로 버리고, runtime에서는
  `valid=false`와 `unsynchronized_input`, `stale_or_future_input` 또는
  `locally_stale_input`으로 무효 발행한다.

현재 구현은 timestamp 이력에서 가장 가까운 state를 찾거나 FT 시각으로 state를
보간하지 않는다. 또한 AFT broadcaster의 stamp는 ROS controller update 시각이므로
CAN sensor의 실제 acquisition 지연을 직접 표현하지 않는다. 따라서 DDS jitter,
callback 순서, 서로 다른 발행 주기와 sensor 내부 지연에 따른 오차가 남을 수 있다.

### 본격 구현 전 측정 체크리스트

다음 측정은 정지, 저속 free-space, 예상 최대 속도의 free-space, 제한된 실제 접촉
순서로 수행한다. SBC/controller/AFT를 재시작한 뒤에도 반복하여 실행별 편차를
확인한다.

| 완료 | 확인 항목 | 기록값/판정 |
|---|---|---|
| [ ] | ObserverInput과 AFT stamp가 같은 SBC ROS clock 기준인지 확인 | stamp 생성 위치와 clock source |
| [x] | PC-SBC Chrony 상태 확인 | `^* .17`, bound `0.041183 ms`, `GO` |
| [ ] | source 발행률과 timestamp gap 확인 | Hz, p95/p99/max gap |
| [ ] | accepted pair의 `sync_error_ms` 분포 확인 | p50/p95/p99/max |
| [ ] | collector rejection 비율 확인 | `sync_rejections / ft_callbacks` |
| [ ] | runtime invalid reason별 횟수 확인 | unsynchronized/stale/future/local stale |
| [ ] | 속도별 sync error와 residual force의 상관 확인 | q/dq, sync error, residual |
| [ ] | 실행 및 재부팅 반복성 확인 | run별 p99/max와 rejection 비율 |
| [ ] | free-space 오검출 영향 확인 | `valid=true` false contact 0회 |
| [ ] | controlled-contact 검출 지연 확인 | source stamp 기준 검출 latency |

수집된 NPZ의 accepted pair 오차는 다음처럼 확인한다.

```bash
python3 -c 'from pathlib import Path; import numpy as np; p=max(Path.home().glob(".ros/ft_fb_leaderarm/data/*.npz"), key=lambda x:x.stat().st_mtime); e=np.load(p)["sync_error_ms"]; print(p); print({"samples":e.size,"p50_ms":float(np.percentile(e,50)),"p95_ms":float(np.percentile(e,95)),"p99_ms":float(np.percentile(e,99)),"max_ms":float(e.max())})'
```

같은 episode의 JSON에서 callback과 rejection 수를 확인한다.

```bash
jq '{ft_callbacks,sync_rejections,invalid_rejections,duplicate_state_rejections}' \
  /home/vision/.ros/ft_fb_leaderarm/data/<episode>.json
```

### 개선 착수 조건과 순서

다음 중 하나가 재현되면 timestamp pairing 개선을 구현한다.

- `sync_rejections`가 반복적으로 발생해 필요한 262.5 Hz 유효 표본을 유지하지
  못한다.
- accepted pair의 p99가 3 ms 제한에 지속적으로 근접한다.
- 빠른 free-space 동작에서 sync error와 residual/false contact가 함께 증가한다.
- 실행 또는 재부팅마다 timing 분포가 크게 달라진다.

구현 순서는 다음으로 고정한다.

1. ObserverInput을 짧은 bounded deque에 보관하고 각 FT stamp와 가장 가까운
   state를 선택한다.
2. collector와 runtime observer가 같은 pairing helper와 동일한 3 ms gate를
   사용하게 하여 학습/실행 차이를 막는다.
3. out-of-order callback, 3 ms 경계, stale/future stamp, duplicate sequence,
   매칭 실패를 포함하는 최소 단위 테스트를 추가한다.
4. nearest-state 방식으로도 속도 연관 residual이 남을 때만 q, dq와 pose의
   timestamp interpolation을 추가한다.
5. 일정한 offset이 남으면 AFT hardware acquisition timestamp 지원 또는 실측한
   sensor delay 보정을 검토한다.

원인을 확인하지 않고 `max_sync_error_ms`나 `max_source_age_ms`만 늘리지 않는다.
매칭되는 state가 없을 때는 기존처럼 fail-closed(`valid=false`)를 유지한다.

## 권장 error budget

아래 값은 측정 전 임시 목표이며 최종 acceptance는 실제 sensor/contact SNR로
확정한다.

| 오차 원인 | 권장 예산 |
|---|---:|
| zero 반복성과 session drift | 0.3~0.5 N 이하 |
| 정지 noise와 spike | 1 N gate에 충분한 여유를 남길 것 |
| model/generalization/sync/frame 합계 | 전체 최대 force-vector 1 N 이하 |

## 점검 명령

```bash
ros2 topic info /aft_sensor2/wrench --verbose
timeout 15s ros2 topic hz /aft_sensor2/wrench
ros2 topic echo /aft_sensor2/wrench --once

ros2 topic info /contact_state/observer_input --verbose
timeout 15s ros2 topic hz /contact_state/observer_input
ros2 topic echo /contact_state/observer_input --once
```

결과와 관련 artifact는 [failure log](failure_log.md)에 연결한다.
