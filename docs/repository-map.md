
# Repository map

## Purpose

이 ROS 2 패키지는 물리 AFT 센서의 free-space wrench를 수집·학습하고,
contact observation과 오른팔 leader feedback을 제공한다.

이 문서는 변경할 코드의 시작 위치를 찾기 위한 지도다.
사용자용 실행 및 설명 문서는 `document/`에 있다.

전체 데이터 흐름은 [README](../README.md#문제-정의-육하원칙), 단계별 운용 흐름은
[flow](../document/flow.md)를 먼저 확인한다.

## Change routes

| 변경 대상 | 시작 위치 | 함께 확인할 위치 |
|---|---|---|
| 목표와 합격 기준 | `docs/acceptance-contract.md` | `document/TODO_LIST.md`, `document/flow.md` |
| 빌드와 설치 | `CMakeLists.txt`, `package.xml` | `scripts/`, `launch/`, `config/` |
| FT 데이터 수집 | `ft_fb_leaderarm/collector_node.py` | `config/collector.yaml`, `launch/collect_free_space*.launch.py` |
| 데이터·feature 계약 | `ft_fb_leaderarm/contract.py` | `ft_fb_leaderarm/validate_dataset.py`, `test/test_contract.py` |
| 모델과 학습 | `ft_fb_leaderarm/model.py`, `ft_fb_leaderarm/train_ablation.py` | `scripts/ft_free_space_train`, `test/test_model_bundle.py` |
| Contact observer | `ft_fb_leaderarm/observer_node.py` | `config/observer.yaml`, `launch/ft_contact_observer.launch.py` |
| Contact 정량 평가 | `ft_fb_leaderarm/contact_evaluation.py` | `scripts/ft_contact_evaluate`, `test/test_contact_evaluation.py` |
| IL contact 계약 검증 | `ft_fb_leaderarm/il_contact_verification.py` | `scripts/ft_il_contact_verify`, `test/test_il_contact_verification.py` |
| IL episode 저장 검증 | `ft_fb_leaderarm/il_episode_verification.py` | `/home/vision/chem_UMI-FT_ACP/UMIFT_Data/wired_collection/Python/chem_acp_raw_data_collection_lowhz.py`, 관련 테스트 |
| Observer runtime/FREE 평가 | `ft_fb_leaderarm/observer_runtime.py` | `scripts/ft_observer_runtime_evaluate`, `test/test_observer_runtime.py` |
| Feedback 분석과 승인 | `ft_fb_leaderarm/feedback_analysis.py`, `ft_fb_leaderarm/feedback_authorization.py` | `scripts/ft_feedback_analyze`, `scripts/ft_feedback_onset_evaluate`, `scripts/ft_feedback_authorize`, 관련 테스트 |
| Leader teleoperation | `src/single_impedance_*.cpp`, `include/ft_fb_leaderarm/` | `launch/ft_feedback_leader_teleop.launch.py`, `config/single_impedance_leader_damping.yaml`, `test/test_teleop_integration.py` |
| Leader intent/smoothing | `src/intent_trajectory_generator.cpp`, `src/single_impedance_pose_publisher.cpp` | `config/single_impedance_leader_damping.yaml`, `test/test_intent_trajectory_generator.cpp`, `document/stabilization_teleoperation/` |
| Feedback IL 수집 GUI | `launch/ft_feedback_leader_data_collection.launch.py` | UMI recorder와 `../fb_leaderarm` GUI |
| 데이터 수집 GUI | `scripts/ft_free_space_collection_gui.py` | `launch/collect_free_space_gui.launch.py` |
| 자동 검증 | `docs/verification.md`, `test/` | `CMakeLists.txt`의 `BUILD_TESTING` 영역 |
| 사용자 운용 문서 | `README.md`, `document/` | 동작 변경 시 함께 갱신 |
| 실패·문제 기록 | `document/problem/README.md` | 해당 문제 파일, 관련 코드·테스트·artifact |

## External boundaries

| 경계 | 위치 | 이 패키지에서 사용하는 것 |
|---|---|---|
| Contact message | `/home/vision/contact_pipeline_ws` | `contact_observer_msgs/ContactObservation` |
| Feedback Leader Arm | `../fb_leaderarm` | IL 수집 GUI와 Chrony helper |
| UMI recorder | `/home/vision/chem_UMI-FT_ACP` | IL episode recorder와 설정 |
| AFT driver/controller | SBC의 `aft_can_hardware`와 Doosan workspace | wrench, hardware zero-set, follower state |

외부 경계는 이 저장소에서 임의로 수정하지 않는다. 계약 변경이 필요하면 해당 저장소의
소스·테스트와 이 패키지의 설정·문서를 함께 확인한다.

## Source of truth

코드 동작은 소스, 설정, launch 파일, 테스트를 기준으로 판단한다.
문서와 코드가 다르면 실제 코드와 테스트를 확인하고 같은 변경에서 문서를 갱신한다.
