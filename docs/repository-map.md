
# Repository map

## Purpose

이 ROS 2 패키지는 물리 AFT 센서의 free-space wrench를 수집·학습하고,
contact observation과 오른팔 leader feedback을 제공한다.

이 문서는 변경할 코드의 시작 위치를 찾기 위한 지도다.
사용자용 실행 및 설명 문서는 `document/`에 있다.

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
| Observer runtime/FREE 평가 | `ft_fb_leaderarm/observer_runtime.py` | `scripts/ft_observer_runtime_evaluate`, `test/test_observer_runtime.py` |
| Feedback 분석과 승인 | `ft_fb_leaderarm/feedback_analysis.py`, `ft_fb_leaderarm/feedback_authorization.py` | `scripts/ft_feedback_analyze`, `scripts/ft_feedback_authorize`, `test/test_feedback_analysis.py`, `test/test_feedback_authorization.py` |
| Leader teleoperation | `src/single_impedance_*.cpp`, `include/ft_fb_leaderarm/` | `launch/ft_feedback_leader_teleop.launch.py`, `config/single_impedance_leader_damping.yaml`, `test/test_teleop_integration.py` |
| 데이터 수집 GUI | `scripts/ft_free_space_collection_gui.py` | `launch/collect_free_space_gui.launch.py` |
| 자동 검증 | `docs/verification.md`, `test/` | `CMakeLists.txt`의 `BUILD_TESTING` 영역 |
| 사용자 운용 문서 | `README.md`, `document/` | 동작 변경 시 함께 갱신 |

## Source of truth

코드 동작은 소스, 설정, launch 파일, 테스트를 기준으로 판단한다.
문서와 코드가 다르면 실제 코드와 테스트를 확인하고 같은 변경에서 문서를 갱신한다.
