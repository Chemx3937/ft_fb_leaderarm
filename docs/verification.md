# Verification

## Selection

- 문서만 변경하면 diff를 검토하고 문서 검증을 실행한다.
- 소스, 설정, launch, 빌드 동작을 변경하면 빌드와 전체 테스트를 실행한다.
- 하드웨어 동작은 소프트웨어 검증을 먼저 수행하고, 사용자 승인 없이 실행하지 않는다.

## Gate map

현재 상태는 [TODO](../document/TODO_LIST.md) 한 곳에서 관리한다. 아래 표는
[acceptance contract](acceptance-contract.md)의 각 gate를 실행 가능한 검증과 evidence로
연결한다. 전체 운용 명령은 [command runbook](../document/command.md)을 중복 작성하지
않고 해당 절에 연결한다.

| Gate | Software check | Operational check | 승인 | Evidence |
|---|---|---|---|---|
| `FS-01` | [contract test](../test/test_contract.py) | [zero·수집 절차](../document/free_space_wrench_data_collection.md#6-fixed-zero-pose와-leader-정렬-확인) | 필요 | episode metadata, zero diagnostics |
| `FS-02` | [dataset test](../test/test_dataset_validation.py) | [dataset validator](../document/command.md#6-데이터-검증) | 불필요 | `dataset_validation_*.json` |
| `FS-03`, `FS-04` | [model tests](../test/test_model_bundle.py) | [학습·비교](../document/command.md#7-5개-모델-학습) | 불필요 | `ablation_report.json`, `metadata.json` |
| `FS-05`, `FS-06` | [runtime test](../test/test_observer_runtime.py) | [runtime/FREE 평가](#observer-runtime-and-free-evidence) | 필요 | `observer_runtime_*.json` |
| `FS-07` | [GUI integration test](../test/test_teleop_integration.py) | [수집 GUI](../document/free_space_wrench_data_collection.md#8-pc에서-ft-collector와-전용-gui-실행) | 필요 | GUI integration 결과, dataset/model report |
| `CO-01`, `CO-02` | [detector contract test](../test/test_contract.py) | [observer-only 검증](../document/command.md#9-운용-허용-모델-observer-only-검증) | 필요 | topic capture, diagnostics |
| `CO-03` | [standalone launch](../launch/ft_contact_observer.launch.py) | [observer-only 검증](../document/command.md#9-운용-허용-모델-observer-only-검증) | 필요 | launch 결과, diagnostics |
| `CO-04` | [contact evaluator test](../test/test_contact_evaluation.py) | [ground-truth 평가](#contact-ground-truth-evidence) | 필요 | `contact_evaluation_*.json` |
| `CO-05` | [IL contract test](../test/test_il_contact_verification.py) | [collection/inference 평가](#il-contact-contract) | 필요 | 모드별 `il_contact_*.json` |
| `FB-01` | [feedback analysis test](../test/test_feedback_analysis.py) | [feedback-OFF evidence](../document/command.md#10-feedback-off-실기-evidence) | 필요 | analysis JSON, 원본 CSV |
| `FB-02` | [onset test](../test/test_feedback_analysis.py) | [onset 평가](#feedback-onset) | 필요 | `feedback_onset_*.json` |
| `FB-03` | [feedback analysis test](../test/test_feedback_analysis.py) | [40% 단계 검증](../document/command.md#12-40-실행) | 필요 | staged analysis와 확정된 진동 metric report |
| `FB-04` | [authorization tests](../test/test_feedback_authorization.py) | [OFF→40%→100% 승인](../document/command.md#11-off--40-자동-분석과-승인) | 필요 | authorization JSON, 원본 CSV hash |
| `FB-05` | [episode test](../test/test_il_episode_verification.py) | [IL test episode](../document/command.md#16-통합-feedback-il-gui-test-episode) | 필요 | `il_episode_verification_*.json` |

## Documentation

```bash
git status --short
git diff --check
git diff
```

## Build and test

```bash
cd /home/vision/dualarm_ws
source /opt/ros/humble/setup.bash
source /home/vision/contact_pipeline_ws/install/setup.bash
colcon build --symlink-install --packages-select ft_fb_leaderarm
source /home/vision/dualarm_ws/install/setup.bash
export PYTHONPATH=/home/vision/venv_act/lib/python3.10/site-packages:$PYTHONPATH
colcon test --packages-select ft_fb_leaderarm --event-handlers console_direct+
colcon test-result --test-result-base build/ft_fb_leaderarm --verbose
```

현재 PC의 CTest는 system `pytest`를 사용하지만 PyTorch는 `venv_act`에 있으므로
전체 테스트 전에 위 `PYTHONPATH`가 필요하다. `contact_observer_msgs`는
`contact_pipeline_ws` 환경에서 제공된다.

## Observer runtime and FREE evidence

사용자 승인 아래 observer가 실행 중일 때만 다음 passive 평가를 실행한다. 같은
report에서 `FS-05` runtime과 `FS-06` FREE residual을 판정한다.

```bash
ros2 run ft_fb_leaderarm ft_observer_runtime_evaluate \
  --duration-s 10 --output /absolute/path/observer_runtime.json
```

## Contact ground-truth evidence

관측 CSV와 독립 장치로 기록한 같은 시계의 `start_s,end_s` 접촉 구간 CSV를
사용한다. 미확정 합격값은 실행자가 명시하며 기존 report를 덮어쓰지 않는다.

```bash
ros2 run ft_fb_leaderarm ft_contact_evaluate -- \
  --observation-csv /absolute/path/observations.csv \
  --ground-truth-csv /absolute/path/contact_intervals.csv \
  --min-precision VALUE --min-recall VALUE \
  --max-onset-latency-ms VALUE --max-release-latency-ms VALUE \
  --output /absolute/path/contact_evaluation.json
```

## IL contact contract

사용자 승인 아래 IL collection 또는 inference 경로가 실행 중일 때 passive
verifier를 각 모드에 한 번씩 실행한다. 두 report 모두 canonical config, 단일
publisher, 해당 consumer와 유효 message capture를 통과해야 한다.

```bash
ros2 run ft_fb_leaderarm ft_il_contact_verify -- \
  --mode collection \
  --recorder-config /absolute/path/recorder.yaml \
  --policy-config /absolute/path/policy_runner.yaml \
  --output /absolute/path/il_contact_collection.json

# Policy inference 실행 중에는 mode와 output만 바꿔 다시 실행한다.
# --mode inference --output /absolute/path/il_contact_inference.json
```

## Feedback onset

확정된 기준값과 controlled CONTACT가 3회 이상 포함된 feedback-enabled leader CSV를
사용한다.

```bash
ros2 run ft_fb_leaderarm ft_feedback_onset_evaluate -- \
  --csv /absolute/path/leader_teleop.csv \
  --max-rise-time-ms VALUE --max-torque-step-nm VALUE \
  --output /absolute/path/feedback_onset.json
```

## IL episode

저장된 test episode를 읽기 전용으로 검증한다. `--expected-stage`는
`0.0`, `0.40`, `1.00` 중 실제 수집 단계와 같은 값을 사용한다.
Recorder에는 선택 모델의 SHA-256과 같은 `--model-sha256`, 실제 stage와 같은
`--feedback-gain-scale-contract`를 전달해야 한다.

```bash
ros2 run ft_fb_leaderarm ft_il_episode_verify -- \
  --episode /absolute/path/episode_000 \
  --model /absolute/path/model.ts \
  --expected-stage 0.40 \
  --output /absolute/path/il_episode_verification.json
```

필수 저장 항목이 하나라도 없으면 `FAIL`이며 해당 episode는 `FB-05` evidence로
인정하지 않는다.

완료 보고에는 실행한 명령, 통과·실패 결과, 실행하지 못한 하드웨어 검증을 기록한다.
