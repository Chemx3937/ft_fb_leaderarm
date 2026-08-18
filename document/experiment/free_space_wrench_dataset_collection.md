# Free-space wrench 학습 dataset 수집 실험

- 수행일: 2026-08-19
- 대상: 오른팔 follower, `/aft_sensor2`, feedback-OFF leader teleoperation
- 범위: FAST 상태의 완전 무접촉 task 3회와 다양한 궤적 10회

## 목적

로봇이 물체와 접촉하지 않고 움직일 때 physical AFT가 측정하는 6축 wrench를
robot state `[q,dq,causal-qdd]`와 함께 `262.5 Hz`로 저장한다. 모델은 이를 이용해
free-space wrench를 예측하며 observer의 contact wrench는 다음 계약을 사용한다.

```text
contact_wrench = bias-removed physical FT raw - predicted free-space wrench
```

task 궤적만 학습했을 때 다른 경로에 과적합되는 것을 줄이기 위해 task 3개와
위치·자세·속도 범위를 나눈 diverse 10개를 서로 독립적인 hardware zero-set으로
수집했다.

## 안전·데이터 계약

- 시작 자세: `[5.5, 52.0, 112.0, 28.0, -107.0, -35.0] deg`
- tool/payload/controller/AFT frame을 전 episode에서 바꾸지 않는다.
- cold start한 AFT는 첫 formal 수집 전 120분 warm-up을 한 번 수행한다.
- 각 episode 전에 기준 자세에서 hardware zero-set을 하고 고유 `zero_set_id`를 쓴다.
- `CURRENT/SLOW`는 정렬에만 사용하고 저장하지 않는다. `FAST`만 기록한다.
- 모든 동작은 완전 무접촉이며 cable이 당겨지지 않는 작업공간에서 수행한다.
- robot/leader 이동 키는 운용자가 주변 안전을 확인한 뒤 직접 누른다.

## 실행 명령

### PC 공통 환경

```bash
source /opt/ros/humble/setup.bash
source /home/vision/contact_pipeline_ws/install/setup.bash
source /home/vision/dualarm_ws/install/setup.bash
source /home/vision/venv_act/bin/activate

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=7
export ROS_LOCALHOST_ONLY=0
export CYCLONEDDS_URI=file:///home/vision/cyclonedds_dualarmPC.xml
export ROS_LOG_DIR=/tmp/ft_fb_leaderarm_ros_logs

export FT_DATA_DIR=/home/vision/.ros/ft_fb_leaderarm/datasets/right_diverse10_20260819
export FT_PAYLOAD_ID=right_tool_m2p1kg_oz0p17m_v1
export FT_CONTROLLER_HASH=bae_r_v2_c113eabf7e13_ca07ae197213
mkdir -p "${FT_DATA_DIR}"
```

PC-SBC 시계와 입력을 확인한다.

```bash
sudo -v
/home/vision/dualarm_ws/src/fb_leaderarm/scripts/dualarm_chrony_mode.sh status

ros2 topic info /contact_state/observer_input --verbose
ros2 topic info /aft_sensor2/wrench --verbose
timeout 15s ros2 topic hz /contact_state/observer_input
timeout 15s ros2 topic hz /aft_sensor2/wrench
```

### SBC 장비 실행과 zero-set

robot driver, 일반 V2 impedance controller, hand driver와 AFT driver 명령은
[실행 명령](../command.md)의 2~4절을 따른다. AFT를 새로 시작했을 때 500 Hz를
한 번 명시한다.

```bash
ros2 topic pub --once /aft_sensor2/sample_rate_setting \
  std_msgs/msg/Int32 "{data: 500}"
```

각 episode 전에 기준 자세·정지·무접촉을 확인한 뒤 zero-set한다.

```bash
ros2 topic info /aft_sensor2/bias_setting --verbose
ros2 launch aft_can_hardware aft_zero_set2.launch.py \
  sensor_name:=aft_sensor2
```

### Collector/GUI와 episode

각 zero-set마다 ID만 바꿔 통합 launch를 새로 실행한다.

```bash
ros2 launch ft_fb_leaderarm collect_free_space_gui.launch.py \
  output_dir:="${FT_DATA_DIR}" \
  zero_set_confirmed:=true \
  zero_set_id:=tare_YYYYMMDD_01 \
  payload_id:="${FT_PAYLOAD_ID}" \
  controller_config_hash:="${FT_CONTROLLER_HASH}" \
  start_teleop:=true
```

GUI가 `VERIFIED / IDLE / IDLE / OK`일 때 다음 순서로 수행한다.

```text
1 → ARMED 확인 → c → CURRENT 확인 → t → SLOW 정렬 완료
→ o → FAST/RECORDING 확인 → 무접촉 궤적
→ 완전 정지 → s → 2
```

종료는 GUI에서 `q` 한 번, `SHUTDOWN Done` 확인, launch terminal에서
`Ctrl+C` 한 번 순서다. `start_teleop:=true`는 startup에서 leader를 follower
자세로 실제 ALIGN할 수 있지만 follower command는 발행하지 않는다.

### 저장 파일과 dataset 검증

최근 JSON은 Python 표준 기능으로 확인할 수 있다.

```bash
LATEST_JSON="$(ls -1t "${FT_DATA_DIR}"/right_free_space_*.json | head -1)"
LATEST_NPZ="${LATEST_JSON%.json}.npz"

python3 -c 'import json,sys; m=json.load(open(sys.argv[1])); print(json.dumps({k:m.get(k) for k in ("accepted","zero_set_id","record_only_teleop_state","samples","duration_s","actual_hz","max_record_gap_ms","sync_rejections","invalid_rejections")},indent=2))' "${LATEST_JSON}"

unzip -t "${LATEST_NPZ}"
```

3개 이상의 독립 zero group을 모은 뒤 validator를 실행한다. output 파일이 이미
있으면 덮어쓰지 않으므로 새 이름을 사용한다.

```bash
source /home/vision/venv_act/bin/activate
ros2 run ft_fb_leaderarm ft_free_space_validate -- \
  --data-dir "${FT_DATA_DIR}" \
  --seed 7 \
  --output "${FT_DATA_DIR}/dataset_validation_v1.json"
```

## 수집 방법

### FAST-only task pilot 3개

동일한 모방학습 task를 서로 다른 hardware zero-set에서 3회 수행했다. 각 episode는
FAST 상태에서만 16~22초 기록했으며 접촉 전에 종료했다.

### Diverse 10개

한 episode에 모든 동작을 넣지 않고, 전체 10개가 합쳐서 상태 공간을 덮도록
역할을 나눴다.

| zero-set | 주 궤적 |
|---|---|
| `div01` | 좌우 왕복, J1 변화 중심 |
| `div02` | 가까이/멀리, J2/J3 변화 중심 |
| `div03` | 위/아래 왕복 |
| `div04` | 손목 J4/J5/J6 양방향 회전 |
| `div05` | Cartesian X 왕복 |
| `div06` | Cartesian Y 왕복 |
| `div07` | Cartesian Z 왕복 |
| `div08` | 대각선·곡선 3차원 이동 |
| `div09` | 위치 이동과 손목 회전 결합 |
| `div10` | 실제 task와 유사한 무접촉 변형 경로 |

각 episode는 시작/종료 정지 구간, 양방향 반복, 느림/보통/조금 빠른 속도와 완만한
가감속을 포함했다. `FAST`는 teleop 상태 이름이며 최대 속도 지시가 아니다.

## 결과

### Task pilot

| zero-set | 파일 | 시간 [s] | samples | 실제 Hz | 최대 gap [ms] | 결과 |
|---|---|---:|---:|---:|---:|---|
| `tare_20260819_02` | `right_free_space_20260819_034917` | 16.495 | 4,331 | 262.504 | 4.014 | PASS |
| `tare_20260819_03` | `right_free_space_20260819_040748` | 21.775 | 5,717 | 262.503 | 4.037 | PASS |
| `tare_20260819_04` | `right_free_space_20260819_041307` | 20.655 | 5,423 | 262.503 | 4.021 | PASS |

- 합계: 3 episodes, 3 groups, 15,471 samples, 58.925초
- 세 파일 모두 FAST-only, `accepted=true`, sync/invalid rejection 0이다.
- pilot 5개 ablation 중 `history_mlp`가 선택됐지만 validation/test 최대 force-vector
  오차가 `4.295/2.733 N`으로 `1 N` 기준을 실패했다.
- runtime p99/max는 `0.0261/0.0489 ms`로 통과했다.

### Diverse 10개

| zero-set | 파일 | 시간 [s] | samples | 실제 Hz | 최대 gap [ms] | sync reject | 결과 |
|---|---|---:|---:|---:|---:|---:|---|
| `div01` | `044348` | 58.674 | 15,403 | 262.501 | 4.034 | 0 | PASS |
| `div02` | `044927` | 62.148 | 16,315 | 262.502 | 4.029 | 6 | PASS |
| `div03` | `045441` | 64.746 | 16,997 | 262.503 | 4.027 | 0 | PASS |
| `div04` | `050000` | 82.788 | 21,733 | 262.502 | 4.025 | 0 | PASS |
| `div05` | `050511` | 78.730 | 20,668 | 262.505 | 4.027 | 3 | PASS |
| `div06` | `050941` | 81.248 | 21,329 | 262.505 | 4.037 | 0 | PASS |
| `div07` | `051410` | 80.342 | 21,091 | 262.503 | 6.000 | 0 | PASS |
| `div08` | `051928` | 117.274 | 30,786 | 262.505 | 5.001 | 1 | PASS |
| `div09` | `052432` | 109.256 | 28,681 | 262.503 | 6.001 | 0 | PASS |
| `div10` | `053045` | 153.582 | 40,317 | 262.505 | 6.006 | 1 | PASS |

- 합계: 10 episodes, 10 independent zero groups, 233,320 samples,
  888.788초(14분 48.8초)
- validator split: train/validation/test `6/2/2 groups`
- payload, controller hash, FT frame, observer frame와 zero pose가 모두 일치했다.
- sync reject 11개는 저장 전에 제외됐다. 보존 sample의 최대 sync error는 모두
  3 ms 이내였고 record gap은 모두 10 ms 이하였다.
- 공식 validator 결과는 PASS다.

## 제외 데이터와 문제 연결

- `right_free_space_20260819_034238.npz`: 이전/새 collector가 동시에 같은 파일을
  기록해 내부 metadata와 외부 JSON이 다르고 두 array 압축이 손상됐다. 원본은
  삭제하지 않고 pilot에서 제외했다. 상세 기록은
  [FT-20260808-01의 FAST pilot 절](../problem/FT-20260808-01.md#2026-08-19-fast-only-task-pilot와-5개-ablation)에 있다.
- `right_free_space_20260819_025951.npz`: SLOW smoke이며 CRC 오류가 있어 제외했다.
- 첫 SLOW episode의 74.757초 길이 초과:
  [FT-20260811-04](../problem/FT-20260811-04.md)
- legacy zero-set 종료 교착:
  [FT-20260808-02](../problem/FT-20260808-02.md)
- zero 기준 자세 부호 불일치:
  [FT-20260808-03](../problem/FT-20260808-03.md)
- AFT 설정 1000 Hz와 실제 500 Hz 갱신 불일치:
  [FT-20260808-04](../problem/FT-20260808-04.md)
- launch Ctrl-C cleanup 중복 SIGINT:
  [FT-20260808-05](../problem/FT-20260808-05.md)
- Chrony helper 상대경로 오류:
  [FT-20260811-01](../problem/FT-20260811-01.md)
- `feedback_source:=off` YAML type 오류:
  [FT-20260811-02](../problem/FT-20260811-02.md)
- startup leader 자동 ALIGN 안내 누락:
  [FT-20260811-03](../problem/FT-20260811-03.md)

## Artifact와 현재 상태

- task pilot:
  `/home/vision/.ros/ft_fb_leaderarm/datasets/right_task3_pilot_20260819/`
- diverse 10:
  `/home/vision/.ros/ft_fb_leaderarm/datasets/right_diverse10_20260819/`
- diverse validator:
  `/home/vision/.ros/ft_fb_leaderarm/datasets/right_diverse10_20260819/dataset_validation_v1.json`
- rejected pilot model/report:
  `/home/vision/.ros/ft_fb_leaderarm/models/right_task3_pilot_20260819/`

현재 3개 task episode와 10개 diverse episode를 합친 최종 13-group dataset 생성,
validator 재실행과 5개 ablation 재학습은 아직 수행하지 않았다. 이 단계는 저장된
파일만 사용하는 offline 작업이며 robot/AFT driver가 필요 없다.

