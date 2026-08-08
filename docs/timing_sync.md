# PC-SBC 시계 동기화와 FT sample 시간 정렬

## 1. 이 문서의 목적

이 시스템에는 흔히 모두 "동기화"라고 부르지만 실제로는 서로 다른 두 문제가
있다.

| 구분 | 맞추는 대상 | 현재 기준 | 해결 수단 |
|---|---|---:|---|
| PC-SBC 시계 동기화 | 두 컴퓨터가 생각하는 현재 시각 | 상대 오차 bound 1 ms 이하 | Chrony direct NTP |
| ObserverInput-AFT sample 정렬 | 어떤 robot state와 FT 측정값이 같은 순간의 값인지 | stamp 차이 3 ms 이하 | header stamp pairing gate |

첫 번째는 두 컴퓨터의 **시계 눈금**을 맞추는 문제다. 두 번째는 같은 눈금으로
기록된 여러 데이터 중에서 **같은 순간의 sample**을 고르는 문제다. Chrony가
정상이어도 sample pairing이 잘못될 수 있고, pairing 코드가 있어도 PC와 SBC의
시계가 다르면 source age 판정이 잘못될 수 있다.

이 문서는 현재 장비 구성을 기준으로 한다.

| 역할 | 현재 값 |
|---|---|
| 일반 PC hostname/IP | `vision` / `192.168.112.2` |
| SBC hostname/IP | `gene2` / `192.168.112.17` |
| PC 유선 interface | `enp8s0` |
| SBC SSH | `vision@192.168.112.17` |
| ROS domain | `ROS_DOMAIN_ID=7` |
| ObserverInput | `/contact_state/observer_input`, 약 1,000 Hz |
| 물리 FT | `/aft_sensor2/wrench`, 약 1,000 Hz publish; collector/runtime 262.5 Hz |

주소나 interface가 바뀌면 이 문서의 명령과 Chrony helper 기본값을 함께 검토한다.

## 2. 먼저 구분할 용어

- **source timestamp**: 데이터를 만든 쪽에서 `header.stamp`에 기록한 시각이다.
- **receive time**: 메시지가 PC callback에 실제 도착한 시각이다.
- **clock offset**: 같은 실제 순간을 PC와 SBC 시계가 서로 다르게 표시하는 정도다.
- **network latency**: 메시지가 네트워크를 통과하는 데 걸린 시간이다.
- **jitter**: latency 또는 callback 실행 간격이 매번 달라지는 현상이다.
- **sample mismatch**: 서로 다른 물리 시점의 state와 wrench를 한 쌍으로 사용한
  상태다.
- **stale**: 메시지가 허용한 시간보다 오래되어 더 이상 안전하게 쓸 수 없는
  상태다.

Chrony는 clock offset을 줄인다. DDS가 만든 network latency나 jitter를 없애지는
않는다. FT pairing은 timestamp 차이를 제한하지만 두 컴퓨터의 시계를 직접
조정하지는 않는다.

---

## 3. 문제 A: 일반 PC와 SBC의 시계 동기화

### 3.1 육하원칙

| 질문 | 현재 시스템의 답 |
|---|---|
| 누가 | 일반 PC `vision`과 SBC `gene2` |
| 언제 | 실제 robot data 수집, contact observer, teleoperation 실행 전과 SBC 재부팅 후 |
| 어디서 | `192.168.112.2`와 `192.168.112.17` 사이의 전용 유선망 |
| 무엇을 | 두 Linux system clock의 상대 오차 |
| 어떻게 | PC가 SBC를 direct Chrony/NTP source로 선택하고 상대 오차 bound를 검사 |
| 왜 | SBC source stamp를 PC의 `now()`와 비교하는 stale/future 안전 판정을 정확히 하기 위해 |

### 3.2 왜 teleoperation에 필요할까

현재 데이터와 명령 흐름은 다음과 같다.

```text
SBC /TorqueRtR
  └─ ObserverInput(header.stamp=SBC 시각)
                         │ DDS
                         ▼
PC FT Contact Observer
  └─ ContactObservation(header.stamp=SBC source 시각)
                         │
                         ▼
PC leader teleoperation
  └─ PC now() - SBC source stamp로 freshness 확인

PC leader teleoperation
  └─ PoseStamped(header.stamp=PC 시각) ──DDS──> SBC impedance controller
```

PC의 contact observer와 leader teleoperation은 다음과 같은 계산을 한다.

```text
source_age = PC의 현재 ROS 시각 - SBC가 기록한 source stamp
```

PC 시계가 SBC보다 8 ms 빠르면 실제 network 지연이 없더라도 `source_age`가
8 ms 오래된 것처럼 보인다. 반대로 PC 시계가 느리면 정상 메시지가 미래에서 온
것처럼 보일 수 있다. 이 때문에 PC와 SBC의 상대 clock error를 먼저 제한한다.

현재 leader teleoperation의 `PoseStamped`에는 PC timestamp가 들어가지만, SBC의
현재 task command callback은 pose를 읽고 header stamp를 이용한 command-age
판정을 하지 않는다. 따라서 Chrony의 주된 안전 효과는 command 전송 자체보다
SBC에서 시작된 observer/contact 데이터의 stale/future 판정에 있다.

### 3.3 현재 Chrony 구조

일반 운용 중 PC는 public NTP source를 사용한다. Robot 운용을 시작할 때만
PC가 SBC를 임시 direct source로 선택한다.

```text
평상시
  public NTP ──> PC

dualarm Chrony ON
  SBC Chrony/NTP server ──전용 유선망──> PC Chrony client
          192.168.112.17                192.168.112.2
                   PC sources 표시에서 ^*로 선택

dualarm Chrony OFF
  SBC runtime source 제거
  SBC의 PC 접근 ACL 차단
  PC가 원래 public NTP source로 복귀
```

이 전환은
[dualarm_chrony_mode.sh](../../fb_leaderarm/scripts/dualarm_chrony_mode.sh)가
담당한다. 이 helper는 persistent Chrony 설정 파일을 수정하지 않고 runtime
상태만 바꾼다.

### 3.4 `on`이 실제로 하는 일

`on`의 순서는 다음과 같다.

1. 실행 host가 `vision`인지, SBC 경로가 `enp8s0`인지, IP와 SSH 대상이 예상값인지
   확인한다.
2. PC가 현재 선택한 public NTP source와 정상 `Leap status`를 기록한다.
3. SSH로 SBC hostname이 `gene2`인지 확인한다.
4. SBC에서 `chronyc allow 192.168.112.2/32`를 실행하여 이 PC의 NTP 접근만
   임시 허용한다.
5. PC Chrony에 SBC를 다음 runtime source로 추가한다.

```text
server 192.168.112.17 iburst prefer xleave minpoll 0 maxpoll 0
```

6. `burst 8/12`로 초기 표본을 빠르게 모으고, 이후 1초 polling으로 수렴시킨다.
7. 최대 60초 동안 direct relative checker가 통과하기를 기다린다.
8. 모두 통과하면 `$XDG_RUNTIME_DIR/fb_leaderarm_dualarm_time_on` marker에 ON
   상태와 이전 public source를 기록한다. `XDG_RUNTIME_DIR`가 없으면
   `/run/user/$UID`를 사용한다.

`xleave`는 interleaved NTP measurement를 사용해 네트워크 왕복 측정 오차를 줄이는
설정이다. `prefer`는 SBC source를 우선 선택하게 한다. `minpoll 0`, `maxpoll 0`은
1초 간격 polling을 뜻한다.

### 3.5 1 ms를 어떻게 판정하는가

[check_chrony_relative_sync.py](../../fb_leaderarm/scripts/check_chrony_relative_sync.py)는
단순히 `chronyc tracking`의 offset 한 값만 보지 않는다. PC가 SBC와 직접 측정한
다음 보수적 bound를 계산한다.

```text
relative_error_bound
  = abs(NTP offset)
  + peer_delay / 2
  + peer_dispersion

통과 조건: relative_error_bound <= 1.0 ms
```

checker는 계산 전에 다음 계약도 모두 확인한다.

- PC의 `chronyc sources -n`에서 `192.168.112.17`이 정확히 한 번 나타난다.
- marker가 `^*`여서 SBC가 실제 선택된 server source다.
- `Reach`가 0이 아니고 마지막 sample이 5초보다 오래되지 않았다.
- NTP remote/local address가 각각 `.17`과 `.2`다.
- NTP v4, UDP port 123, server mode, 정상 stratum과 `Leap status=Normal`이다.
- `NTP tests`가 모두 통과하고 interleaved mode가 활성화됐다.
- TX/RX timestamping이 `Kernel` 또는 `Hardware`다.
- 유효한 direct NTP 응답을 최소 8개 받았다.
- PC가 실제 SBC reference를 추적하고 PC stratum이 SBC보다 정확히 1 높다.
- SSH로 읽은 SBC tracking과 PC가 측정한 SBC reference/stratum이 일치한다.

어느 하나라도 실패하면 fail-closed로 종료하고 robot workflow를 시작하지 않는
것이 현재 계약이다.

[check_chrony_sync.py](../../fb_leaderarm/scripts/check_chrony_sync.py)는 양쪽의
공통 upstream 기준 absolute UTC 상태를 보는 별도 진단 도구다. PC가 SBC를 직접
측정한 1 ms relative gate를 대신하지 않는다.

### 3.6 실행 방법

일반 PC에서 실행한다.

```bash
cd /home/vision/dualarm_ws/src/fb_leaderarm
sudo -v

./scripts/dualarm_chrony_mode.sh on
./scripts/dualarm_chrony_mode.sh status
```

정상 핵심 표시는 다음과 같다.

```text
^* 192.168.112.17
GO: dualarm Chrony mode ON; relative bound <= 1.0 ms.
GO: dualarm Chrony status is internally consistent.
```

`status`도 SBC ACL과 direct measurement를 검사하므로 먼저 `sudo -v`로 credential을
준비해야 한다. 암호 입력이 불가능한 비대화형 shell에서 이를 생략하면 selected
source가 정상이어도 전체 status 검사가 실패할 수 있다. SBC의 `chronyc accheck`
검사에서도 원격 sudo 설정에 따라 SBC 암호를 추가로 요구할 수 있다.

수동 checker만 다시 실행하려면 다음 명령을 사용한다.

```bash
source /opt/ros/humble/setup.bash
source /home/vision/dualarm_ws/install/setup.bash

ros2 run fb_leaderarm check_chrony_relative_sync.py \
  --source 192.168.112.17 \
  --local-address 192.168.112.2 \
  --remote vision@192.168.112.17 \
  --max-relative-offset-ms 1.0 \
  --timeout-sec 5.0
```

작업을 마치고 PC를 원래 public NTP 모드로 돌릴 때는 다음을 실행한다.

```bash
./scripts/dualarm_chrony_mode.sh off
./scripts/dualarm_chrony_mode.sh status
```

`off`는 PC의 runtime SBC source를 제거하고, SBC에서 `.2/32` 접근을 차단한 뒤,
PC가 기존 public NTP source를 다시 선택할 때까지 확인한다.

### 3.7 언제 다시 해야 하는가

- SBC를 종료하거나 재부팅한 뒤 첫 robot 운용 전
- PC 또는 SBC의 유선 IP/interface가 바뀐 뒤
- `stale_or_future_input`이나 source age 음수가 반복될 때
- `^* 192.168.112.17`, `Reach`, ACL 또는 1 ms 검사 중 하나가 실패할 때
- data 수집 또는 실제 feedback을 시작하기 전 `status`가 GO가 아닐 때

SBC 재부팅은 runtime ACL을 없앨 수 있다. 기존 ON marker만 믿지 말고 다음 순서로
복구한다.

```bash
sudo -v
./scripts/dualarm_chrony_mode.sh off
./scripts/dualarm_chrony_mode.sh on
./scripts/dualarm_chrony_mode.sh status
```

### 3.8 현재 `ft_fb_leaderarm`에서 주의할 점

기존 `fb_leaderarm` V2 launch들은 `chrony_preflight=true`일 때 node 시작 전에
relative checker를 실행한다. 현재 `ft_fb_leaderarm`의 launch에는 이 preflight가
내장돼 있지 않다. 따라서 현재는 operator가 위 helper의 `on`과 `status`를 먼저
실행해야 한다.

향후 production launch를 본격 구현할 때는 checker를 새로 만들지 말고 기존
`check_chrony_relative_sync.py`를 재사용하여 시작 전 fail-closed gate를 추가한다.
실제 PC와 SBC를 사용하는데 preflight를 끄는 옵션은 허용하지 않고, 단일 PC
offline test에서만 우회를 허용한다.

### 3.9 2026-08-08 현재 상태

operator가 sudo credential을 준비한 뒤 full status를 실행했다. PC source는
`^* 192.168.112.17`, `Reach=377`, `LastRx=1 s`였고 direct measurement는 다음과
같다.

| 항목 | 결과 |
|---|---:|
| absolute offset | `0.004206 ms` |
| peer delay / dispersion | `0.073864 / 0.000045 ms` |
| direct relative error bound | `0.041183 ms` |
| NTP tests / interleaved | `111 111 1111 / Yes` |
| TX/RX timestamping | `Kernel / Kernel` |
| helper 판정 | `valid=true`, `GO` |

source 목록의 `+/- 7.865 ms`는 upstream root uncertainty이며 PC-SBC direct
relative bound `0.041183 ms`와 같은 값이 아니다. 현재 1 ms Chrony gate는
통과했다. SBC 재부팅 또는 수집 재시작 전에는 다음 status를 다시 확인한다.

```bash
cd /home/vision/dualarm_ws/src/fb_leaderarm
sudo -v
./scripts/dualarm_chrony_mode.sh status
```

---

## 4. 문제 B: ObserverInput과 AFT sample 시간 정렬

### 4.1 육하원칙

| 질문 | 현재 시스템의 답 |
|---|---|
| 누가 | SBC `/TorqueRtR`의 robot state와 SBC AFT broadcaster의 physical wrench |
| 언제 | dataset 수집 callback과 262.5 Hz runtime inference 때마다 |
| 어디서 | 일반 PC의 collector와 FT contact observer 내부 |
| 무엇을 | FT 측정 시각과 가장 가까운 robot state sample |
| 어떻게 | 두 `header.stamp` 차이와 freshness gate 확인 |
| 왜 | 서로 다른 자세/속도의 state와 wrench를 결합해 가짜 residual을 만들지 않기 위해 |

### 4.2 source와 timestamp

```text
SBC impedance controller
  /contact_state/observer_input
  header.stamp = RT robot state를 가져온 SBC ROS 시각
  rate         = 약 1,000 Hz

SBC AFT broadcaster
  /aft_sensor2/wrench
  header.stamp = ros2_control update 시각
  publish rate = 약 1,000 Hz
  force update = 약 500 Hz 추정, 연속 force 중복률 약 50%
```

AFT stamp는 CAN sensor 자체의 hardware acquisition timestamp가 아니다. 따라서
ROS stamp가 잘 맞아도 CAN read와 controller update 사이의 일정한 sensor delay가
남을 수 있다.

### 4.3 현재 pairing 방식

[collector_node.py](../ft_fb_leaderarm/collector_node.py)는 FT callback이 실행될 때
가장 최근에 수신한 ObserverInput 하나를 가져온다.

```text
FT callback
  ├─ latest ObserverInput 존재/valid/frame 확인
  ├─ abs(ft_stamp - state_stamp) <= 3 ms 확인
  ├─ state local receive age <= 20 ms 확인
  └─ 통과하면 한 training row 저장
```

[observer_node.py](../ft_fb_leaderarm/observer_node.py)는 262.5 Hz timer마다 최신
ObserverInput과 최신 FT를 읽고 다음을 확인한다.

```text
abs(ft_stamp - state_stamp) <= 3 ms
-2 ms <= PC now - source stamp <= 20 ms
local monotonic receive age <= 20 ms
source_sequence가 직전 inference와 다름
```

설정값은 다음과 같다.

| 설정 | 현재 값 | 의미 |
|---|---:|---|
| `max_sync_error_ms` | 3.0 ms | 두 source stamp 사이 허용 차이 |
| `max_source_age_ms` | 20.0 ms | source/local freshness 제한 |
| `clock_future_tolerance_ms` | 2.0 ms | PC 기준 약간 미래인 stamp 허용 범위 |

### 4.4 mismatch가 발생하면

- collector는 해당 조합을 저장하지 않고 `sync_rejections`를 증가시킨다.
- runtime observer는 내부 history/contact detector를 초기화하고
  `ContactObservation.valid=false`를 발행한다.
- 잘못된 wrench를 feedback에 적용하지 않고 다음 정상 pair를 기다린다.
- leader teleoperation은 invalid, stale, model-not-ready 또는 FREE 상태에서
  feedback torque를 0으로 만든다.

즉, 현재 설계는 timing 품질이 나쁘면 출력을 계속 만들어 내는 방식이 아니라
사용 가능한 sample 수를 줄이는 fail-closed 방식이다.

### 4.5 현재 방식의 한계

현재 코드는 timestamp가 가장 가까운 state를 history에서 찾지 않는다. callback
시점의 `latest` 하나만 사용하므로 다음 상황에서 정상 후보가 있어도 pair가
거부될 수 있다.

- DDS message 도착 순서가 잠시 뒤바뀜
- 1,000 Hz state와 1,000 Hz FT publish의 phase가 달라짐
- PC callback scheduling이 지연됨
- AFT의 실제 acquisition과 ROS stamp 사이에 고정 delay가 존재함
- 빠른 동작에서 1~3 ms의 state 차이가 residual을 크게 만듦

측정 항목, NPZ의 `sync_error_ms` 확인 명령, 개선 착수 조건과 구현 순서는
[FT sensor 확인 목록](FTsensor_check_list.md#observerinput-aft-시간-정렬-검증과-향후-개선)에
정리돼 있다.

향후 개선 순서는 bounded state deque에서 nearest stamp 선택, collector/runtime의
공통 pairing helper, 경계조건 test, 필요할 때만 interpolation, 마지막으로 AFT
hardware timestamp 또는 실측 delay 보정이다. 원인 확인 없이 3 ms나 20 ms 제한만
늘리지 않는다.

### 4.6 2026-08-08 정지 상태 실측

고정 초기 자세에서 10초간 두 source를 동시에 측정했다.

| 항목 | 결과 |
|---|---:|
| ObserverInput / FT publish rate | `1000.035 / 1000.001 Hz` |
| latest callback pair p99/max | `0.830 / 1.833 ms` |
| offline nearest pair p99/max | `0.625 / 0.831 ms` |
| state / FT source age p99 | `1.071 / 0.668 ms` |
| state / FT timestamp gap p99 | `1.368 / 1.011 ms` |

현재 latest pairing도 3 ms 제한에 여유 있게 들어오므로 nearest-state pairing은
구현하지 않는다. 빠른 free-space 동작에서 residual과 sync error의 상관이
재현될 때 다시 판단한다. FT wrench는 1 kHz로 publish되지만 연속 force 3축의
완전 중복률이 약 50%였다. 이는 AFT hardware `read()`가 controller update마다
CAN frame 하나를 읽고 force/torque의 마지막 값을 함께 publish하는 구현과
일치한다. 실제 hardware acquisition timestamp는 여전히 제공되지 않는다.

별도 30초 정지 run에서는 latest pair p99/max가 `1.287/2.625 ms`였다. 3 ms
제한은 통과하지만 첫 10초 run의 max `1.833 ms`보다 커서, 실제 free-space
episode에서도 `sync_rejections`와 max 값을 계속 기록한다.

---

## 5. 두 동기화 문제의 관계

### 5.1 서로 대신할 수 없다

| 상황 | Chrony 결과 | sample pairing 결과 |
|---|---|---|
| PC와 SBC 시계가 8 ms 다름 | 실패 | SBC에서 찍힌 state/FT끼리의 차이는 작을 수 있지만 PC source-age는 틀림 |
| 시계는 0.1 ms 이내지만 FT와 state가 5 ms 차이 | 성공 | 3 ms gate 실패 |
| stamp 차이는 1 ms지만 network가 30 ms 멈춤 | 성공 가능 | source/local stale 20 ms gate 실패 |
| topic이 PC에서 보이지 않음 | Chrony와 무관할 수 있음 | 먼저 DDS/network 설정 확인 |

현재 AFT와 ObserverInput은 모두 SBC에서 stamp를 찍으므로 두 source stamp의 **상호
차이**에는 PC clock offset이 직접 들어가지 않는다. 그러나 PC runtime이
`PC now - SBC stamp`로 freshness를 검사하므로 Chrony는 여전히 필요하다.

만약 향후 AFT publisher를 PC로 옮기면 ObserverInput은 SBC clock, AFT는 PC
clock으로 stamp를 찍게 된다. 그때는 Chrony 오차가 3 ms sample pairing budget에
직접 포함된다.

### 5.2 숫자 1 ms, 3 ms, 20 ms의 차이

```text
1 ms  : PC-SBC 시계 자체의 보수적 상대 오차 bound
3 ms  : state sample과 FT sample을 같은 순간으로 인정하는 최대 stamp 차이
20 ms : PC가 메시지를 너무 오래된 데이터로 판단하는 freshness 한계
2 ms  : PC clock 기준 미래 stamp 허용 한계
```

1 ms Chrony gate를 통과했다고 해서 3 ms pairing이 보장되는 것은 아니다. 반대로
3 ms pair가 만들어졌다고 해서 PC에서 20 ms freshness 계산이 정확하다는 뜻도
아니다.

---

## 6. 전체 실행 순서

### 6.1 통신 설정

PC와 SBC에서 같은 `RMW_IMPLEMENTATION`, `ROS_DOMAIN_ID=7`,
`ROS_LOCALHOST_ONLY=0`을 사용한다. CycloneDDS interface 설정도 각 장비의 유선망과
일치시킨다. 이것은 topic을 서로 발견하고 전달하기 위한 **통신 설정**이며 Chrony
시계 동기화와는 별개다.

### 6.2 Chrony 준비

PC에서 다음을 실행하고 `status` GO를 확인한다.

```bash
cd /home/vision/dualarm_ws/src/fb_leaderarm
sudo -v
./scripts/dualarm_chrony_mode.sh on
./scripts/dualarm_chrony_mode.sh status
```

### 6.3 SBC source 시작

SBC에서 robot driver, impedance controller, AFT broadcaster 순으로 시작한다.
ObserverInput과 AFT가 정상 발행되는지 확인한다.

```bash
ros2 topic info /contact_state/observer_input --verbose
ros2 topic info /aft_sensor2/wrench --verbose
timeout 15s ros2 topic hz /contact_state/observer_input
timeout 15s ros2 topic hz /aft_sensor2/wrench
```

### 6.4 PC observer와 teleoperation 시작

collector 또는 observer를 시작한 뒤 diagnostics에서 다음을 확인한다.

- `sync_rejections`
- `invalid_publications`
- `last_invalid_reason`
- `prediction_age_ms`
- `observer_latency_ms`
- leader의 `observer_source_age_ms`, `observer_local_age_ms`

`valid=false`를 무시하거나 `chrony_preflight=false`로 실제 장비 gate를 우회하지
않는다.

---

## 7. 증상별 원인 구분

| 증상 | 먼저 볼 문제 | 이유 |
|---|---|---|
| PC에서 topic 자체가 안 보임 | ROS_DOMAIN_ID/CycloneDDS/network | 시계가 달라도 topic discovery는 가능함 |
| `stale_or_future_input` 반복 | PC-SBC Chrony와 실제 network 지연 | `PC now - source stamp`가 범위를 벗어남 |
| `locally_stale_input` 반복 | DDS/callback stall | local monotonic receive age가 큼 |
| `unsynchronized_input` 반복 | FT-state sample pairing | 두 source stamp 차이가 3 ms 초과 |
| Chrony `Reach=0` | NTP ACL/route/SBC 상태 | PC가 SBC NTP 응답을 받지 못함 |
| Chrony source가 `^*`가 아님 | ON 수렴 또는 source 선택 실패 | PC가 SBC를 실제 기준으로 사용하지 않음 |
| `status`가 sudo 오류로 중단 | credential 미준비 | 먼저 `sudo -v` 필요 |
| 재부팅 뒤 ON marker만 남음 | runtime ACL/state 불일치 | `off -> on -> status`로 복구 |
| 빠른 동작에서만 residual 증가 | sample mismatch/sensor delay | nearest pairing과 속도 상관 분석 필요 |

## 8. 구현 시 지켜야 할 경계

- PC-SBC Chrony와 FT-state pairing을 하나의 timeout 값으로 합치지 않는다.
- DDS receive time은 packet stall 진단에 사용하고 물리 sample time을 대신하게 하지
  않는다.
- collector와 runtime observer가 동일한 pairing 규칙을 사용하게 한다.
- time gate 실패 시 기존 fail-closed 동작을 유지한다.
- current `ft_fb_leaderarm` production launch에 Chrony preflight를 추가할 때는
  검증된 `fb_leaderarm` checker를 재사용한다.
- AFT가 어느 host에서 stamp를 생성하는지 바뀌면 3 ms budget을 다시 검증한다.
- 모든 실측 결과는 `sync_error_ms`, rejection count, source/local age와 함께
  보존한다.
