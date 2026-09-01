# Legacy 문서 안내

이 디렉터리는 현재 운용 절차로 사용하지 않는 설계 후보와 종료된 개선 캠페인을
보존한다. 과거에 무엇을 검토했고 현재 무엇으로 대체됐는지 확인하기 위한 기록이다.

| Legacy 문서 | 기존 방식 | 현재 방식 |
|---|---|---|
| [free_space_wrench_candidate_architectures.md](free_space_wrench_candidate_architectures.md) | static/dynamic/history MLP, LSTM, GRU 5개 후보 비교 | [physical gravity + 32-sample multiscale ridge](../free_space_wrench_model_architecture.md)를 현재 운용 모델로 선택 |
| [free_space_wrench_model_improvement_campaign.md](free_space_wrench_model_improvement_campaign.md) | 정확도 gate를 통과할 때까지 runtime 사용을 금지한 2026-08 개선 탐색 | 알려진 정확도 한계를 기록한 채 정확한 model/metadata SHA 쌍을 `operator_selected_20260901`로 운용 |

다음 자료는 이 디렉터리로 옮기지 않았다.

- `document/experiment/`: 실행 당시 조건과 결과를 보존하는 evidence다.
- `document/problem/`: 원인·조치·재검증 이력이다.
- `document/stabilization_teleoperation/`: 현재 Smooth teleoperation 구현과 운용
  runbook이 계속 참조하는 설계·검증 문서다.

현재 동작은 코드, [현재 모델 architecture](../free_space_wrench_model_architecture.md),
[전체 흐름](../flow.md), [실행 명령](../command.md)을 기준으로 판단한다.

