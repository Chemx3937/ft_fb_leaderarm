# Leader arm teleoperation 안정화 문서

이 디렉터리는 leader arm teleoperation의 부드럽지 않은 움직임을 분석하고
안정화하는 작업 문서를 관리한다. 구현과 검증 변경은 `smooth_teleop` 브랜치에서만
진행하며 `main`은 안정화 작업 적용 전 상태를 유지한다.

## 문서

- [안정화 설계와 전체 pipeline](leader_arm_teleoperation_stabilization.md)
- [Smooth teleoperation 구현 및 검증](smooth_teleop_implementation.md)

실제 로봇을 움직이는 teleoperation, force feedback 활성화와 데이터 수집 검증은
사용자 승인 후 수행한다.
