# FT sensor 확인 목록

## 원칙

1 N은 모델 오차만의 예산이 아니라 sensor noise, drift, timestamp mismatch,
frame 오차를 모두 포함한 결과 예산이다. sensor 관련 오차가 전체 1 N에
근접하면 어떤 모델도 요구 성능을 안정적으로 만족하기 어렵다.

동일 zero 자세에서 zero 직후 값이 0에 가까운 것은 반복성 증거가 아니다.
매 zero-set 후 여러 검증 자세로 이동한 측정값을 비교한다.

## 체크리스트

| 완료 | 항목 | 방법 | 기록값/판정 |
|---|---|---|---|
| [ ] | warm-up drift | ON 후 0/15/30/60/120분 같은 자세 기록 | session 동안 force 변화, 온도 |
| [ ] | zero 반복성 | 같은 조건에서 zero 10회, 여러 자세 왕복 | group 간 force-vector 분산/최대차 |
| [ ] | power-cycle 반복성 | SBC/AFT 재부팅 전후 반복 | 재부팅 전후 offset 변화 |
| [ ] | 단기 noise | 정지 상태 30~60초 | 축별 std, force norm p95/max |
| [ ] | 장시간 bias | 재-zero 없이 예상 작업 시간 기록 | 시간에 따른 drift slope/max |
| [ ] | 온도 영향 | sensor/주변 온도와 wrench 동시 기록 | 온도-bias 상관 |
| [ ] | 자세 반복성 | 같은 자세를 양 방향에서 접근 | hysteresis |
| [ ] | payload/tool | 체결과 질량 중심 확인 | `payload_id`, 체결 토크, 사진 |
| [ ] | cable strain | 모든 자세에서 cable 간섭 확인 | cable 방향별 offset |
| [ ] | frame/부호 | 알려진 +X/+Y/+Z 힘 적용 | sensor/base 축과 부호 |
| [ ] | lever arm | sensor→TCP translation 측정 | torque 변환 오차 |
| [ ] | timestamp sync | FT와 observer input stamp 비교 | max/p95 sync error |
| [ ] | 발행률/jitter | `ros2 topic hz`, timestamp diff | rate, max gap, drop count |
| [ ] | saturation | 허용 범위 내 하중 후 제거 | rail 여부, offset 복귀 |
| [ ] | overload recovery | 안전 범위 시험 후 정지 기록 | 영점 복귀 시간 |
| [ ] | EMI/CAN | 모터/전원 변화 중 기록 | spike, reconnect, packet loss |
| [ ] | free false contact | 다양한 무접촉 움직임 | activation 0회 |
| [ ] | controlled contact | 제한한 방향/크기 접촉 | 검출/해제/방향/latency |

## 현재 software zero gate

collector와 observer는 다음을 만족하기 전에는 valid inference/수집을 시작하지
않는다.

| 항목 | 현재 값 |
|---|---:|
| 초기 joint 자세 | `[5.5, 52.0, 112.0, 28.0, -107.0, -35.0] deg` |
| joint 자세 허용오차 | 1.0 deg |
| 최대 joint 속도 | 0.02 rad/s |
| 안정 시간 | 1.0 s |
| zero force median norm | ≤ 1.0 N |
| 각 force 축 표준편차 | ≤ 0.20 N |

이 gate는 장시간 drift나 독립 zero-set 반복성을 보장하지 않는다.

## 권장 error budget

아래 값은 측정 전 임시 목표이며 최종 acceptance는 실제 sensor/contact SNR로
확정한다.

| 오차 원인 | 권장 예산 |
|---|---:|
| zero 반복성과 session drift | 0.3~0.5 N 이하 |
| 정지 noise와 spike | 1 N gate에 충분한 여유를 남길 것 |
| model/generalization/sync/frame 합계 | 전체 최대 force-vector 1 N 이하 |

## 점검 명령

```bash
ros2 topic info /aft_sensor2/wrench --verbose
timeout 15s ros2 topic hz /aft_sensor2/wrench
ros2 topic echo /aft_sensor2/wrench --once

ros2 topic info /bae_r/observer_input --verbose
timeout 15s ros2 topic hz /bae_r/observer_input
ros2 topic echo /bae_r/observer_input --once
```

결과와 관련 artifact는 [failure log](failure_log.md)에 연결한다.
