# Leader arm teleoperation 안정화 실행 순서

## 1. 목적

이 문서는 leader arm teleoperation의 부드럽지 않은 움직임을 실제로 해결하고
검증하기 위해 **운용자가 수행할 일을 순서대로** 정리한 runbook이다.

단순히 화면에서 부드러워 보이는 것으로 완료 처리하지 않는다. 같은 task의
`smooth OFF`/`smooth ON` 데이터를 비교하여 command 진동이 감소하고, follower
tracking·접촉 timing·작업 성공률·최대 접촉력이 악화되지 않아야 한다.

모든 구현과 실험 기록은 `smooth_teleop` 브랜치에서 관리한다. 현재 요구사항에 따라
`main`은 안정화 변경 적용 전 기준 버전으로 유지하며, 명시적인 결정 전에는 merge하지
않는다.

## 2. 전체 순서

```text
0. software blocker 제거
  -> 1. 안전 한계와 합격 기준 확정
  -> 2. target PC 배포·빌드·테스트
  -> 3. robot/AFT/observer 준비
  -> 4. logging 정상 여부 확인
  -> 5. feedback OFF에서 smooth OFF/ON A/B
  -> 6. intent parameter 조정
  -> 7. smooth ON 상태에서 feedback OFF -> 40% -> 100%
  -> 8. IL test episode와 motion-quality 비교
  -> 9. 최종 parameter·evidence 동결
```

앞 단계가 실패하면 다음 단계로 진행하지 않는다.

## 3. 0단계 — 실기 전 software blocker 제거

이 단계는 로봇을 움직이지 않는다. 운용자가 직접 코드를 수정할 필요는 없지만, 다음
항목이 완료됐는지 개발자 또는 Codex와 함께 확인해야 한다.

- [x] `smooth_teleop` 브랜치가 `main`과 분리되어 있다.
- [x] 2차 intent generator와 velocity/acceleration 제한이 구현되어 있다.
- [x] 고정 target에서 intent 위치·회전·속도가 수렴하는 회귀 테스트가 있다.
- [x] raw/intent/final command CSV logging이 구현되어 있다.
- [x] 기존 leader YAML 뒤에 로드하는 **별도 smooth teleop YAML overlay**가 생성되어 있다.
- [x] `smooth_teleop_enable:=false`가 기존 command elapsed/slew 동작까지 정확히
  재현한다.
- [x] `contact_observer_msgs` 환경을 source한 target PC에서 전체 build/test가
  통과한다.

마지막 세 항목이 완료되기 전에는 정식 A/B 결과를 만들지 않는다. 특히 같은 YAML에서
값을 계속 바꾸면 baseline을 재현하기 어려우므로 base YAML과 실험 YAML을 분리해야
한다.

개발 workspace에서 확인한다.

```bash
cd /home/chem/ft_fb_leaderarm
git branch --show-current
git status --short
git log --oneline --decorate -3
git diff --stat main..smooth_teleop
```

합격 조건:

- branch 출력이 `smooth_teleop`이다.
- 의도하지 않은 working-tree 변경이 없다.
- `main`에는 intent generator와 안정화 문서가 없다.
- 실험할 commit ID를 기록했다.

## 4. 1단계 — 안전 한계와 A/B 계약 확정

로봇을 켜기 전에 다음 값을 먼저 문서로 확정한다. 미확정 값을 실험 중 임의로 바꾸지
않는다.

| 항목 | 확정할 값 |
|---|---|
| 대상 arm | right |
| task | 기존 logistic box task와 동일한 동작 |
| operator | 동일 운용자 |
| 시작 자세 | `[5.5, 52.0, 112.0, 28.0, -107.0, -35.0] deg` |
| tool/payload | 실제 ID와 질량·CoM |
| controller hash | 실제 실행 controller hash |
| feedback 단계 | OFF, 40%, 100% |
| A/B 조건 | A=`smooth OFF`, B=`smooth ON` |
| 조건별 episode 수 | `N = ______` |
| controlled-contact 최대 힘 | `______ N` |
| 비상 정지 담당자와 방법 | `________________` |

환경 변수의 placeholder를 승인된 값으로 바꾼다. 최대 접촉력이 미정이면 controlled
contact와 feedback 승인을 진행하지 않는다.

```bash
export FT_MAX_CONTACT_FORCE_N=REPLACE_WITH_APPROVED_TASK_LIMIT
```

### 4.1 고정 안전 기준

다음 운용 기준은 실험 중 완화하지 않는다. 정식 모델 승격용 `FS-03`의
p95/p99/hard max `1/1/2 N`은 별도로 유지한다.

- FREE residual force: p95 `<= 1.2 N`, p99 `<= 1.5 N`, hard max `<= 2.5 N`
- FREE false CONTACT: 0회
- leader pose step: `<= 1 deg`
- velocity reversal: `<= 8 Hz`
- invalid/stale/FREE에서 reflected feedback torque: 정확히 0
- torque clip, watchdog, joint/workspace limit 유지

### 4.2 이번 실험 전에 사용자가 확정해야 하는 개선 기준

아래 값은 아직 acceptance contract의 open decision이다. 측정 후 유리한 값으로
바꾸지 말고 A/B 전에 합격값을 적는다.

| 지표 | 기존 contact-observer 중앙값 | smooth ON 합격값 |
|---|---:|---:|
| command position jitter RMS | 0.603 mm | `______` |
| command position HF power | 0.015% | `______` |
| command rotation jitter RMS | 0.164 deg | `______` |
| command rotation HF power | 0.103% | `______` |
| actual position jitter RMS | 0.247 mm | `______` |
| joint-vector jitter RMS | 0.047 deg | `______` |
| command/actual tracking lag | 99.607 ms | `______` |
| task 성공률 | 별도 기록 필요 | `______` |
| 완료시간 허용 변화 | 별도 결정 필요 | `______` |
| 최대 접촉력 | 승인 안전 한계 이하 | `______` |

기존 수치는
`/home/chem/UMI-FT/analysis_results/logistic_box_motion_quality_comparison/report.md`
기준이다. actual pose source가 다른 데이터끼리 직접 비교할 때는 source 차이를 결과에
명시한다. 이번 smooth OFF/ON 비교에서는 양쪽 모두 같은
`controller_current_pose_se3`를 사용한다.

### 4.3 2026-08-24 feedback-OFF 확인 A/B 고정 기준

운용자가 다음 기준과 동작을 데이터 취득 전에 승인했다.

- leader 정지 중 command 속도 `> 30 mm/s`가 `0.1 s` 이상 지속: `0회`
- `3 Hz` 이상 position raw→command power 잔존율: `<= 25%`
- `10 Hz` 이상 position raw→command power 잔존율: `<= 5%`
- raw-intent position 차이: p95 `<= 20 mm`, max `<= 30 mm`
- Smooth ON/OFF 수행시간 차이: `<= 20%`
- 접촉·비정상 동작: `0회`, 운전자 체감 지연 허용 가능

양쪽 조건은 FAST에서 `5 s` 정지, 약 `30 mm` 왕복, `2 s` 정지, 손목 약 `10 deg`
왕복, 마지막 `5 s` 정지 후 PAUSE 순서로 실행한다. 이 기준은 feedback-OFF smooth
확인용이며 feedback 진동 전달의 최종 `FB-03` 기준을 대신하지 않는다.

확인 A/B 결과는 다음과 같다.

| 항목 | Smooth OFF | Smooth ON | 기준 | 결과 |
|---|---:|---:|---:|---|
| FAST 시간 | `51.646 s` | `49.084 s` | 차이 `<= 20%` | PASS (`4.96%`) |
| `3 Hz` 이상 raw→command 잔존율 | `79.688%` | `4.647%` | ON `<= 25%` | PASS |
| `10 Hz` 이상 raw→command 잔존율 | `72.744%` | `0.188%` | ON `<= 5%` | PASS |
| 정지 중 `>30 mm/s`가 `>=0.1 s` 지속 | `0회` | `0회` | `0회` | PASS |
| raw-intent position 차이 p95/max | `0/0 mm` | `2.127/5.139 mm` | `<=20/30 mm` | PASS |

- OFF CSV: `logs/leader_teleop_right_20260824_035338.csv`, SHA-256
  `07b655c749e0ef6a15311fb57778b8efb65d1cc12674b577c9a293b31817a066`
- ON CSV: `logs/leader_teleop_right_20260824_035714.csv`, SHA-256
  `83ee867d8fc4e7fa35d59ced1cb2e4c816ff0c4af01e96ac6ecabf89a8b3aaf6`
- 운전자 확인: 접촉 없음, Smooth ON 지연 허용 가능
- 판정 범위: **feedback OFF, 작은 저속 free-space 확인에 한정해 PASS**. 빠른 동작,
  다양한 궤적, 실제 접촉, feedback ON과 IL episode로 일반화하지 않는다.

## 5. 2단계 — target PC 배포·빌드·테스트

hardware PC workspace에 실험할 `smooth_teleop` commit을 배포한 뒤 확인한다.

```bash
cd /home/vision/dualarm_ws/src/ft_fb_leaderarm
git branch --show-current
git log -1 --oneline
git status --short
```

그다음 [공통 환경과 빌드 절차](../command.md#0-모든-pc-터미널의-공통-설정과-주의사항)를
따라 `contact_observer_msgs` 환경까지 source하고 빌드·전체 테스트를 실행한다.

```bash
cd /home/vision/dualarm_ws
source /opt/ros/humble/setup.bash
source /home/vision/contact_pipeline_ws/install/setup.bash

colcon build --symlink-install --packages-select ft_fb_leaderarm \
  --cmake-force-configure \
  --cmake-args -DBUILD_TESTING=ON -DPython3_EXECUTABLE=/usr/bin/python3

source /home/vision/dualarm_ws/install/setup.bash
export PYTHONPATH=/home/vision/venv_act/lib/python3.10/site-packages:${PYTHONPATH}
colcon test --packages-select ft_fb_leaderarm
colcon test-result --verbose
```

launch 인자도 확인한다.

```bash
ros2 launch ft_fb_leaderarm ft_feedback_leader_teleop.launch.py --show-args
ros2 launch ft_fb_leaderarm ft_feedback_leader_data_collection.launch.py --show-args
```

합격 조건:

- build와 전체 test가 실패 없이 끝난다.
- `smooth_teleop_enable`, `leader_config`, `smooth_teleop_config`가 두 launch에 모두 보인다.
- 설치된 package의 smooth YAML 경로와 Git commit ID를 기록했다.

## 6. 3단계 — robot, AFT, observer 준비

이 단계부터 실제 장비가 움직일 수 있다. 주변 사람과 장애물을 정리하고 E-stop을 바로
누를 수 있는 상태에서 **사용자가 실기 실행을 명시적으로 승인한 뒤** 진행한다.

순서는 바꾸지 않는다.

1. [SBC robot driver 실행](../command.md#2-sbc-robot-driver)
2. [SBC V2 impedance controller 실행](../command.md#3-sbc-v2-impedance-controller)
3. [AFT 실행과 오른팔 hardware zero-set](../command.md#4-sbc-aft-on과-hardware-zero-set)
4. payload, controller hash, sensor frame, `zero_set_id` 기록
5. [운용 허용 모델 observer-only 검증](../command.md#9-운용-허용-모델-observer-only-검증)

observer-only 상태에서 다음을 확인한다.

- publish rate `>= 262.5 Hz`
- invalid, stale, deadline miss 0회
- 무접촉에서 residual p95 `<= 1.2 N`, p99 `<= 1.5 N`, hard max `<= 2.5 N`
- 무접촉 false CONTACT 0회
- 동일 topic에 observer publisher가 두 개 이상 존재하지 않음

하나라도 실패하면 teleoperation을 시작하지 않는다.

## 7. 4단계 — logging 계약 확인

짧은 feedback-OFF pilot을 한 번만 수행하여 CSV header와 상태 기록을 확인한다. 정식
A/B episode에는 포함하지 않는다.

필수 leader CSV 신호:

```text
task_raw_*
task_intent_*
task_command_*
tau_grav_*
tau_damp_*
tau_fb_*
observer_*
smooth_teleop_enabled
feedback_gain_scale_contract
```

IL dataset으로 정식 비교할 때는 다음 신호도 같은 episode에 있어야 한다.

```text
robot/command_quat_pose_se3
robot/controller_current_pose_se3
robot/joint_deg
각 신호의 timestamp
```

raw, intent, command 중 하나라도 없거나 timestamp가 단조 증가하지 않으면 A/B를
시작하지 않는다.

## 8. 5단계 — feedback OFF에서 smooth OFF/ON A/B

force feedback 영향을 분리하기 위해 반드시 feedback OFF부터 시작한다. observer
구독은 유지하되 reflected gain만 0으로 둔다.

실험 YAML 절대 경로를 지정한다. 아래 파일은 0단계에서 생성·검증된 실제 파일이어야
한다.

```bash
export FT_LEADER_CONFIG=/absolute/path/to/single_impedance_leader_damping.yaml
export FT_SMOOTH_CONFIG=/absolute/path/to/single_impedance_leader_smooth_teleop.yaml
test -f "${FT_LEADER_CONFIG}"
test -f "${FT_SMOOTH_CONFIG}"
```

### 8.1 A — 기존 command 경로

```bash
ros2 launch ft_fb_leaderarm ft_feedback_leader_teleop.launch.py \
  leader_config:="${FT_LEADER_CONFIG}" \
  smooth_teleop_config:="${FT_SMOOTH_CONFIG}" \
  model_path:="${FT_MODEL}" \
  zero_set_confirmed:=true \
  zero_set_id:=runtime_smooth_off_01 \
  payload_id:="${FT_PAYLOAD_ID}" \
  controller_config_hash:="${FT_CONTROLLER_HASH}" \
  learned_feedback_enable:=false \
  smooth_teleop_enable:=false
```

### 8.2 B — intent generator 적용 경로

```bash
ros2 launch ft_fb_leaderarm ft_feedback_leader_teleop.launch.py \
  leader_config:="${FT_LEADER_CONFIG}" \
  smooth_teleop_config:="${FT_SMOOTH_CONFIG}" \
  model_path:="${FT_MODEL}" \
  zero_set_confirmed:=true \
  zero_set_id:=runtime_smooth_on_01 \
  payload_id:="${FT_PAYLOAD_ID}" \
  controller_config_hash:="${FT_CONTROLLER_HASH}" \
  learned_feedback_enable:=false \
  smooth_teleop_enable:=true
```

각 run에서 같은 순서를 사용한다.

1. follower와 leader 정렬 상태를 확인한다.
2. `c`로 CURRENT에 진입한다.
3. `t`로 SLOW_SYNC에 진입한다.
4. `SLOW READY`를 확인한 뒤 `o`로 FAST에 진입한다.
5. 사전에 정한 동일 task를 동일한 시작 자세에서 수행한다.
6. 이상 움직임이 있으면 `s`로 즉시 PAUSE한다.
7. 종료 후 CSV와 episode를 조건이 드러나는 새 이름으로 보존한다.

Leader를 정지했는데 follower가 계속 움직이면 정상적인 filter delay로 간주하지 않고
즉시 `s` 또는 E-stop으로 중단한다. [`FT-20260824-01`](../problem/FT-20260824-01.md)과
같은 자발 command가 재현되면 Smooth ON을 다시 실행하지 않는다.

A와 B에서 다음을 동일하게 유지한다.

- operator, task 순서, 시작 자세와 workspace
- payload, controller, robot speed와 impedance 설정
- model과 zero-set 절차
- pre-roll과 기록 시간
- episode 수와 중단 episode 처리 규칙

첫 pilot은 A 다음 B 순서로 수행한다. 정식 비교에서는 순서 효과를 줄이기 위해
`A-B-B-A`처럼 조건 순서를 교차한다. hardware zero-set을 새로 수행했다면 같은
`zero_set_id` group 안에서 A와 B를 모두 수집하고, 한 조건만 있는 zero group은 정식
비교에서 제외한다.

학습 데이터 단위 motion-quality 비교까지 할 경우
[통합 Feedback IL GUI test episode](../command.md#16-통합-feedback-il-gui-test-episode)를
사용하되 A/B를 서로 다른 새 session에 저장하고 각각
`smooth_teleop_enable:=false|true`를 전달한다.

## 9. 6단계 — feedback OFF 결과 판정과 parameter 조정

먼저 leader CSV의 세 신호를 비교하여 원인을 분리한다.

| 관측 결과 | 해석 | 다음 조치 |
|---|---|---|
| raw가 흔들리고 intent/command는 깨끗함 | generator가 artifact를 차단함 | tracking과 조작 지연 확인 |
| raw와 intent가 모두 흔들림 | filter 대역 또는 limit가 부족함 | natural frequency와 limit 조정 |
| intent는 깨끗하지만 command가 흔들림 | 최종 slew/timing 경로 문제 | command limiter 점검 |
| command는 깨끗하지만 actual이 흔들림 | follower controller/tracking 문제 | impedance controller 점검 |
| raw부터 큰 drift/자발 움직임 | gravity/damping 문제 | intent보다 gravity/damping 먼저 조정 |
| raw는 정지했는데 intent/command가 계속 움직임 | generator 불안정 | 즉시 중단하고 software blocker로 복귀 |

parameter는 한 번에 한 종류만 변경한다.

- 조작 지연이 크면 natural frequency를 올리는 방향을 검토한다.
- 3 Hz 이상 jitter가 남으면 natural frequency를 낮추는 방향을 검토한다.
- 순간 acceleration이 크면 acceleration limit를 낮춘다.
- jerk limiter는 `FT-20260824-01`의 자발 운동 원인이므로 다시 활성화하지 않는다.
- 움직임이 지나치게 막히면 어떤 limiter가 지속적으로 활성화되는지 CSV로 확인한다.
- 좁은 공진 peak가 반복 측정된 경우에만 notch filter를 검토한다.

값을 변경할 때마다 다음을 기록하고 5단계를 다시 수행한다.

```text
Git commit
YAML SHA-256
변경 parameter와 이전/새 값
변경 이유
smooth OFF/ON session 경로
결과 report 경로
```

feedback OFF에서 command smoothness와 조작성 기준을 통과하기 전에는 force feedback
40%로 진행하지 않는다.

### 9.1 일반화 stress 검증

실제 IL 운용을 대표하는 다음 세 고정 sequence를 사용하고 각 sequence를 `3회`
반복한다. 모든 run은 정지 구간을 앞뒤에 포함하며, 실제 command 속도 분포로
slow/nominal/fast 조건이 분리됐는지 사후 확인한다.

속도·힘 범위는 기존 IL 데이터
`/data/logistic_box_contact_observer/episode_000~099`의 100개 episode와 278개 contact
onset을 기준으로 정했다. 과거 actual TCP 선속도는 p95 `149.77 mm/s`, 최대
`223.89 mm/s`였고, contact 직전 선속도는 p95 `95.61 mm/s`, 최대 `145.43 mm/s`,
canonical contact force는 p99 `21.79 N`, 최대 `30.70 N`이었다. 이는 안전 한계의
증명이 아니라 현재 장비 검증 범위를 정하기 위한 근거다.

| Sequence | 속도 | 궤적 |
|---|---|---|
| `G1` | `0~75 mm/s` | 직선 왕복 + 정지 + 단일축 회전 |
| `G2` | `75~160 mm/s` | 곡선 + translation/rotation 결합 |
| `G3` | `160~260 mm/s` | 방향 반전 + 급정지 + 다축 결합 |

free-space 검증 선속도 상한은 `260 mm/s`, 각속도 상한은 `60 deg/s`로 둔다.
sequence 속도 등급은 역사 데이터와 같이 command 선속도 p95로 판정한다. `G1`은
p95 `<=75 mm/s`이면서 순간 최대가 `160 mm/s` 미만, `G2`는 p95
`75~160 mm/s`이면서 순간 최대가 `260 mm/s` 이하, `G3`는 p95
`160~260 mm/s`이면서 모든 sample이 `260 mm/s` 이하여야 한다. follower FK 속도
분포는 같은 방식으로 별도 기록한다. 속도 p95 window는 의무적인 시작·종료 정지
구간을 제외한 실제 task 수행 구간으로 고정하고, 마지막 정지는 별도 판정한다.
기존 command hard cap `300 mm/s`는 유지한다. 먼저 `G1/G2/G3`를 각 1회, 총 3회
smoke로 실행하고 모두 통과한 경우에만 각 sequence를 2회씩 추가해 최종 총 9회를
채운다.

2026-08-24 `G1` 첫 시도는 속도 조건이 섞여 **집계에서 제외**했다.

- CSV: `logs/leader_teleop_right_20260824_042247.csv`, SHA-256
  `a91175b6bafd36a3ee742f6942e67457e8f3c2bca2ffca3f73dfe50b36ff6f28`
- FAST `35.515 s`, Smooth ON, feedback gain `0`
- command 선속도 p95/max `74.37/181.38 mm/s`; `75 mm/s` 초과 `1.734 s`,
  `160 mm/s` 초과 `0.372 s`
- follower FK 선속도 p95/max `79.17/186.04 mm/s`
- raw-intent 위치 차이 p95/max `5.904/14.850 mm`, `3/10 Hz` 이상 position
  power 잔존율 `4.531/0.058%`
- 마지막 연속 정지는 약 `3.04 s`로 계획한 `5 s`보다 짧음
- 판정: smoothing 지표는 통과했지만 `G1` 속도·정지 protocol 불충족. 더 느린
  `G1`을 다시 수행하며 이 run은 빠른 sequence 횟수로도 재사용하지 않는다.

2026-08-24 `G1` 재시도는 첫 왕복의 짧은 정지 후 같은 FAST 안에서 정상 sequence를
다시 수행했다. 정상 재시도 구간만 판정해 **G1 smoke 1회로 승인**했다.

- CSV: `logs/leader_teleop_right_20260824_043432.csv`, SHA-256
  `318b014dcc2524f2ba0bf6e9f51a6a51878fa6c1db43773e9c25d03db0f7efa2`
- 정상 재시도 구간: FAST 상대시간 약 `22~54 s`
- command 선속도 p95/max `62.58/103.66 mm/s`, follower FK 선속도 p95/max
  `65.65/95.97 mm/s`
- raw-intent 위치 차이 p95/max `4.991/8.645 mm`, `3/10 Hz` 이상 전체 FAST
  position power 잔존율 `3.145/0.063%`
- 마지막 5초 command 속도 p95/max `4.14/5.53 mm/s`
- 운전자 확인: 접촉 없음, 이상 움직임 없음
- 판정: G1 분포·smoothing·최종 정지 PASS. 순간 `75 mm/s` 초과는 재시도 구간
  `1.198 s`였으며 G2/G3 검증을 대신하지 않는다.

2026-08-24 `G2` 첫 smoke는 **PASS**했다.

- CSV: `logs/leader_teleop_right_20260824_044507.csv`, SHA-256
  `7ced2ca9c142cd3b71735c85d42ad2bc5870bec22949c0136768f91c8c0456e5`
- FAST `48.454 s`, Smooth ON, feedback gain `0`
- command 선속도 p95/max `145.87/234.93 mm/s`, follower FK 선속도 p95/max
  `137.38/234.32 mm/s`
- command/follower 각속도 max `34.04/22.92 deg/s`
- raw-intent 위치 차이 p95/max `11.401/18.966 mm`, `3/10 Hz` 이상 position
  power 잔존율 `13.415/0.076%`
- command→follower FK 추정 lag `166 ms`; lag 보정 tracking error p95
  `8.61 mm` (`G1` 동일 방식 lag `156 ms`)
- 마지막 5초 command 속도 p95/max `4.87/13.23 mm/s`, follower FK 속도
  p95/max `2.71/5.13 mm/s`
- FAST loop gap `>10 ms` 7회, 최대 `61 ms`; G3에서 재확인
- 운전자 확인: 접촉 없음, 이상 움직임 없음
- 판정: G2 속도 분포·smoothing·tracking·최종 정지 PASS. loop gap은 다음 단계
  timing 감시 항목이며 G3에서 증가하거나 체감 이상이 있으면 즉시 중단한다.

2026-08-24 `G3` 첫 smoke는 앞쪽의 잘못 수행한 sequence를 제외하고 정상 수행한
후반부만 판정해 **PASS**했다.

- CSV: `logs/leader_teleop_right_20260824_045444.csv`, SHA-256
  `d0a0e6de792d177a5f8a70936771be64b15bec1b7ee167586f9c8c9dc9f0d719`
- 정상 task 구간: FAST 상대시간 약 `49.4~82.3 s`
- command 선속도 p95/max `160.84/196.37 mm/s`, follower FK 선속도 p95/max
  `163.74/199.22 mm/s`
- command/follower 각속도 max `36.39/17.64 deg/s`
- raw-intent 위치 차이 p95/max `12.593/15.649 mm`, `3/10 Hz` 이상 task
  position power 잔존율 `15.635/0.145%`
- command→follower FK 추정 lag `198 ms`, lag 보정 tracking error p95
  `11.49 mm`
- 마지막 5초 command 속도 p95/max `5.60/7.11 mm/s`, follower FK 속도
  p95/max `7.25/20.45 mm/s`; `30 mm/s` 자발 운동 기준 미발생
- FAST loop gap `>10 ms` 9회, 최대 `23 ms`; G2 최대 `61 ms`보다 악화되지 않음
- linear acceleration limit 활성 비율 `1.072%`, raw pose 범위를 벗어난 command
  overshoot 없음
- 운전자 확인: 접촉 없음, 이상 움직임 없음
- 판정: G3 속도 분포·smoothing·tracking·방향 반전·최종 정지 PASS. 앞쪽 오동작
  구간은 어느 sequence 횟수에도 재사용하지 않는다.

2026-08-24 `G1` 두 번째 반복에서는 운전자가 의도하지 않은 횡이동과 끊기는 듯한
follower 움직임을 보고했다. 이 run은 **집계에서 제외**하고 추가 stress 반복을
중단했다. frame 변환 후에도 수직 왕복에 world lateral displacement
`18.55/14.67 mm`가 있었고, 마지막 정지 중 leader raw부터 횡 drift가 발생했다.
상세 evidence와 미확정 원인은 [`FT-20260824-02`](../problem/FT-20260824-02.md)를
따른다. 원인 분리와 재검증 전에는 G1/G2/G3 추가 반복으로 진행하지 않는다.

### 9.2 follower command 차단 leader-only 진단

`FT-20260824-02`의 횡 drift가 follower/controller와 무관하게 leader raw부터 생기는지
먼저 분리한다. 일반 teleoperation 기본값은 `true`이며, 이 진단에서만 CLI로 follower
`PoseStamped` 발행을 차단한다.

```bash
source /opt/ros/humble/setup.bash
source /home/vision/contact_pipeline_ws/install/setup.bash
source /home/vision/dualarm_ws/install/setup.bash

ros2 run ft_fb_leaderarm ft_fb_leader_single_impedance_teleop --ros-args \
  --params-file "${FT_LEADER_CONFIG}" \
  --params-file "${FT_SMOOTH_CONFIG}" \
  -p side:=right \
  -p feedback_source:="'off'" \
  -p follower_command_publish_enabled:=false
```

시작할 때 leader 자동 정렬은 leader를 움직일 수 있다. 터미널의
`[LEADER ONLY] ... DISABLED`와 status의
`follower_command_publish_enabled=false`를 확인하기 전에는 `c`를 누르지 않는다.
follower가 움직이면 다른 publisher/controller 경로가 있다는 뜻이므로 즉시 `s` 또는
E-stop으로 중단한다. 이 모드에서는 `z`, `r`, `q`를 누르지 않는다.

1. `IDLE`에서 `c → t → SLOW READY → o` 순서로 FAST에 진입한다.
2. 중력보상이 arm 무게를 완전히 지지하지 못하므로 leader를 끝까지 놓지 않고 5초간
   정지한다.
3. 계속 지지한 상태에서 workspace 경계에 닿지 않는 방향으로 수직 `50 mm`를
   `4초 이상`에 이동하고 3초간 정지한 뒤 같은 경로로 복귀한다.
4. 5초간 정지한 뒤 `s`, `Ctrl+C` 순서로 종료한다.
5. CSV의 `follower_command_publish_enabled=0`, workspace clip, leader raw 횡이동을
   함께 판정한다.

leader에서 손을 놓거나 지지력을 의도적으로 줄이는 free-drift 시험은 하지 않는다.
leader가 급격히 처지거나 지그재그·걸림이 느껴지면 그 자리에서 지지하고 즉시 `s`로
중단한다.

첫 run에서는 gravity/damping parameter를 바꾸지 않는다. baseline drift가 재현된 뒤에만
한 종류씩 비교하며, 이 진단을 통과하기 전에는 G1/G2/G3 횟수에 포함하지 않는다.

2026-08-24 첫 leader-only run은 follower command가 전 구간 차단되고 follower가
정지한 상태에서도 leader의 대각선 지그재그와 걸림이 재현됐다. 두 번째 FAST 중
workspace `z=200 mm` 상한에 약 `12.2 s` 머물렀으므로 깨끗한 수직 왕복 run으로는
집계하지 않는다. 정량 evidence는
[`FT-20260824-02`](../problem/FT-20260824-02.md)를 따른다.

로그에서 FAST scale이 약 10초 후 전 축 `1.0`으로 내려간 것은 정상 설계다.
CURRENT/SLOW/PAUSE와 `z`는 leader를 버틸 수 있게
`grav_sync_scale_per_joint`로 보상을 강화하지만, FAST는 조작을 방해하지 않도록
scale `1.0`과 기존 joint별 `grav_gain`을 사용한다. FAST에 sync scale을 유지하는
비교는 하지 않는다.

gravity 영향 비교에서는 FAST에서 `g`를 한 번 눌러 scale 목표만 `0`으로
내린다. `grav_gain`은 변경하지 않으며 실제 scale은 `grav_ramp_sec=1.5 s`
시정수로 점진적으로 감소한다. leader를 계속 지지하고 최소 10초 대기한 뒤,
상태 표시의 전 축 scale이 `0.01` 이하인 것을 확인한 후에만 짧은 수직/곡선
원시 동작을 수행한다. 이상 처짐이나 가속이 있으면 즉시 `s`를 누른다.
`s`는 FAST gravity OFF 상태와 관계없이 sync scale로 자동 복원한다. 복원 중에도
leader를 놓지 않는다.

2026-08-24 G1/G2 ON/OFF 결과, 저속 ON은 걸림을 키웠지만 OFF는 조작 힘을
크게 만들었다. FAST J1~J3를 50%로 낮춘 G1에서도 지그재그는 심하고 조작 힘은
OFF에 가깝게 무거웠다. 균일 75%는 G1에서 평균적으로 가장 나았지만 G2와 축별
진단에서 J1·J2 걸림, 좌우 방향 편차와 하강 저항이 컸다. J1/J2/J3를
50/62.5/75%로 분리하자 앞뒤·좌우는 저속에서만 일부 걸리는 수준으로 개선됐지만
하강 저항은 남았다. J2를 더 낮춘 FAST J1/J2/J3 50/50/75%에서는 다양한 속도와
방향이 전반적으로 개선됐고, 사용자가 현재 중력보상 기준값으로 승인했다. sync
상태의 실제 보상 제품은 유지한다.

```yaml
grav_gain: [0.125, 0.125, 0.3375, 0.1, 0.3, 0.1]
grav_sync_scale_per_joint: [5.0, 6.0, 3.3333333333, 1.5, 2.5, 1.0]
```

이 설정을 현재 FAST 기준값으로 고정한다. 저속 J1·J2 걸림과 하강 방향 편차는
잔여 기계적 특성으로 기록하고 중력보상 반복 튜닝을 중단한다. 다음 단계는 feedback
OFF에서 follower command를 활성화해 intent smoothing이 잔여 흔들림을 follower에
전달하지 않는지 검증하는 것이다.

전 조합을 무작정 늘리지 않고 위 covering set을 동일 시작 자세와 workspace에서
반복한다. 단계는 다음 순서를 바꾸지 않는다.

1. feedback OFF free-space에서 `G1/G2/G3`
2. 현재 운용 허용 모델로 observer-only free-space에서 같은 sequence
3. feedback 40% free-space에서 같은 sequence
4. 40% controlled contact는 slow부터 시작하고 승인된 힘·속도·방향만 사용
5. 40% 통과 후 100%에서 대표 subset 재검증
6. 작은 IL test episode에서 command/actual/joint 기록과 verifier 확인

각 단계에서 raw→intent→command뿐 아니라 follower actual/joint HF, tracking lag/error,
velocity/acceleration limit 활성 비율, 방향 반전 overshoot, 정지 후 자발 운동을 함께
판정한다. feedback ON에서는 leader raw 진동이 커져도 intent/command와 follower
actual/joint로 전달되지 않는지를 별도 transfer metric으로 확인한다.

controlled contact는 접근 속도 `50 mm/s` 이하에서 시작하고 단계 검증 후에도
`125 mm/s`를 넘지 않는다. 목표 contact force는 `20 N` 이하, hard abort 기준은
`25 N`이다. contact 방향, onset/release와 진동 전달 합격 기준은 contact 전에
추가로 고정한다. 하나라도 미확정이면 contact와 feedback gain 승인을 진행하지 않는다.

## 10. 7단계 — smooth ON에서 force feedback 단계 승인

이후 모든 조건은 `smooth_teleop_enable:=true`로 고정한다. 단계 순서는 반드시
다음과 같다.

1. [Feedback OFF 실기 evidence](../command.md#10-feedback-off-실기-evidence)
   - FREE run 3개
   - 승인 힘 이하 controlled-contact run 1개, CONTACT 최소 3회
2. [OFF에서 40% 자동 분석과 승인](../command.md#11-off--40-자동-분석과-승인)
3. [40% 제한 실행](../command.md#12-40-실행)
   - feedback 방향부터 확인
   - FREE run 3개와 controlled-contact run 기록
4. [40%에서 100% 자동 분석과 승인](../command.md#13-40--100-자동-분석과-승인)
5. [100% 제한 실행](../command.md#14-100-제한-실행)

각 launch에 다음 인자를 명시적으로 추가한다.

```text
leader_config:=${FT_LEADER_CONFIG}
smooth_teleop_config:=${FT_SMOOTH_CONFIG}
smooth_teleop_enable:=true
```

다음 중 하나가 보이면 `s` 또는 E-stop으로 즉시 중지한다.

- feedback 방향이 반대임
- leader가 손을 놓았을 때 가속하거나 drift함
- 접촉 시작 순간 torque/pose jump가 발생함
- FREE에서 진동 또는 feedback torque가 발생함
- observer invalid/stale, follower state stale, watchdog 오류
- 승인한 최대 접촉력 초과

40%가 실패하면 gain을 임의로 올리지 않는다. 원인을 수정하고 OFF evidence부터 새로
만든다.

## 11. 8단계 — IL episode와 motion-quality 정식 비교

feedback 단계별로 작은 IL test episode를 먼저 저장하고
`ft_il_episode_verify`가 `passed=true`인지 확인한다. 실패 episode는 학습에 사용하지
않는다.

smooth OFF/ON dataset은 공통 episode 번호와 동일한 actual pose source를 사용해
비교한다.

```bash
export FT_MOTION_QUALITY_SCRIPT=/absolute/path/to/chem_acp_compare_motion_quality.py
export FT_SMOOTH_OFF_DATASET=/absolute/path/to/smooth_off_dataset
export FT_SMOOTH_ON_DATASET=/absolute/path/to/smooth_on_dataset
export FT_SMOOTH_AB_REPORT=/absolute/path/to/smooth_off_vs_on_report

test -f "${FT_MOTION_QUALITY_SCRIPT}"
test -d "${FT_SMOOTH_OFF_DATASET}"
test -d "${FT_SMOOTH_ON_DATASET}"

python "${FT_MOTION_QUALITY_SCRIPT}" \
  --dataset-a "${FT_SMOOTH_OFF_DATASET}" \
  --label-a smooth_off \
  --actual-pose-a controller_current_pose_se3 \
  --dataset-b "${FT_SMOOTH_ON_DATASET}" \
  --label-b smooth_on \
  --actual-pose-b controller_current_pose_se3 \
  --output-dir "${FT_SMOOTH_AB_REPORT}"
```

다음 파일을 함께 검토한다.

```text
report.md
episode_metrics.csv
group_summary.csv
statistical_comparison.csv
axis_group_summary.csv
outlier_episodes.csv
motion_quality_overview.png
tracking_quality.png
normalized_spectra.png
```

판정 순서:

1. timestamp gap, 누락 episode, 중단 run을 먼저 확인한다.
2. command jitter/HF/path excess가 사전 합격 기준을 통과하는지 확인한다.
3. actual과 joint jitter도 같은 방향으로 개선됐는지 확인한다.
4. tracking lag/error가 허용 범위인지 확인한다.
5. task 성공률, 완료시간, 최대 접촉력이 악화되지 않았는지 확인한다.
6. outlier episode를 영상·CSV와 함께 확인한다.

## 12. 9단계 — 최종값 동결과 완료 조건

다음을 모두 만족해야 teleoperation 안정화 완료로 판정한다.

- [ ] target PC 전체 build/test 통과
- [ ] feedback OFF smooth OFF/ON A/B 합격
- [ ] operator가 지연·저항 증가를 허용 가능한 수준으로 확인
- [ ] FREE/invalid/stale feedback torque 0 확인
- [ ] controlled CONTACT timing과 최대 힘 기준 통과
- [ ] OFF -> 40% -> 100% authorization 순서 준수
- [ ] command뿐 아니라 actual/joint smoothness도 합격
- [ ] tracking과 task performance regression 없음
- [ ] 작은 IL episode verifier 통과
- [ ] 최종 YAML, Git commit, model hash, evidence hash 기록

최종 parameter를 확정하면 실험 중간 YAML을 덮어쓰지 말고 승인본을 별도 이름으로
보존한다. 결과 보고서에는 최소한 다음 정보를 남긴다.

```text
Git branch/commit
leader YAML 경로와 SHA-256
model 경로와 SHA-256
payload ID / controller hash / zero_set_id
smooth flag / feedback stage
dataset와 leader CSV 경로
A/B 합격 기준과 실제 결과
중단·실패 episode 목록
hardware 승인자와 실행 날짜
```

현재 요구사항상 `main`은 이전 버전 기준선으로 계속 유지한다. 안정화가 완료되더라도
사용자가 별도로 결정하기 전에는 `smooth_teleop`을 `main`에 merge하지 않는다.

## 13. 관련 문서

- [안정화 설계와 전체 pipeline](leader_arm_teleoperation_stabilization.md)
- [현재 smooth teleop 구현](smooth_teleop_implementation.md)
- [전체 장비 실행 명령](../command.md)
- [검증 기준](../../docs/acceptance-contract.md)
- [검증 명령 선택](../../docs/verification.md)
