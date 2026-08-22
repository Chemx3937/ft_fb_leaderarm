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
- [x] 2차 intent generator와 velocity/acceleration/jerk 제한이 구현되어 있다.
- [x] raw/intent/final command CSV logging이 구현되어 있다.
- [ ] 기존 leader YAML을 기반으로 한 **별도 smooth teleop YAML**이 생성되어 있다.
- [ ] `smooth_teleop_enable:=false`가 기존 command elapsed/slew 동작까지 정확히
  재현한다.
- [ ] `contact_observer_msgs` 환경을 source한 target PC에서 전체 build/test가
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

다음 기존 기준은 완화하지 않는다.

- FREE residual force: p95/p99 `<= 1 N`, hard max `<= 2 N`
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
- `smooth_teleop_enable`과 `leader_config`가 두 launch에 모두 보인다.
- 설치된 package의 smooth YAML 경로와 Git commit ID를 기록했다.

## 6. 3단계 — robot, AFT, observer 준비

이 단계부터 실제 장비가 움직일 수 있다. 주변 사람과 장애물을 정리하고 E-stop을 바로
누를 수 있는 상태에서 **사용자가 실기 실행을 명시적으로 승인한 뒤** 진행한다.

순서는 바꾸지 않는다.

1. [SBC robot driver 실행](../command.md#2-sbc-robot-driver)
2. [SBC V2 impedance controller 실행](../command.md#3-sbc-v2-impedance-controller)
3. [AFT 실행과 오른팔 hardware zero-set](../command.md#4-sbc-aft-on과-hardware-zero-set)
4. payload, controller hash, sensor frame, `zero_set_id` 기록
5. [승인 모델 observer-only 검증](../command.md#9-승인-모델-observer-only-검증)

observer-only 상태에서 다음을 확인한다.

- publish rate `>= 262.5 Hz`
- invalid, stale, deadline miss 0회
- 무접촉에서 residual p95/p99 `<= 1 N`, hard max `<= 2 N`
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
export FT_SMOOTH_CONFIG=/absolute/path/to/smooth_teleop_config.yaml
test -f "${FT_SMOOTH_CONFIG}"
```

### 8.1 A — 기존 command 경로

```bash
ros2 launch ft_fb_leaderarm ft_feedback_leader_teleop.launch.py \
  leader_config:="${FT_SMOOTH_CONFIG}" \
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
  leader_config:="${FT_SMOOTH_CONFIG}" \
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

parameter는 한 번에 한 종류만 변경한다.

- 조작 지연이 크면 natural frequency를 올리는 방향을 검토한다.
- 3 Hz 이상 jitter가 남으면 natural frequency를 낮추는 방향을 검토한다.
- 순간 acceleration이 크면 acceleration/jerk limit를 낮춘다.
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
leader_config:=${FT_SMOOTH_CONFIG}
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
