# Leader arm teleoperation 안정화 문서

이 디렉터리는 leader arm teleoperation의 부드럽지 않은 움직임을 분석하고
안정화하는 작업 문서를 관리한다. 현재 구현과 검증 상태는 branch 이름이 아니라
아래 구현 문서, runbook과 실제 설정을 기준으로 판단한다.

## 문서

- [안정화 설계와 전체 pipeline](leader_arm_teleoperation_stabilization.md)
- [Smooth teleoperation 구현 및 검증](smooth_teleop_implementation.md)
- [운용자가 수행할 안정화 실행 순서](teleoperation_stabilization_runbook.md)

실제 로봇을 움직이는 teleoperation, force feedback 활성화와 데이터 수집 검증은
사용자 승인 후 수행한다.
