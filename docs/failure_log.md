# FT free-space wrench failure log

## 목적

실패를 덮어쓰거나 기억에 의존하지 않고, 같은 조건의 재발 여부와 수정 효과를
추적한다. 문제를 기록할 때는 육하원칙과 artifact 경로를 반드시 포함한다.

## 분류 코드

| 코드 | 분류 | 예시 |
|---|---|---|
| `SENSOR` | AFT hardware/통신 | CAN drop, saturation, EMI spike |
| `ZERO` | tare/warm-up | zero 불안정, 장시간 drift |
| `SYNC` | timestamp/rate | FT-state sync 초과, source stale |
| `DATA` | collector/dataset | 접촉 혼입, gap, metadata mismatch |
| `MODEL` | 학습/정확도 | validation/test 1 N 초과, overfit |
| `RUNTIME` | inference/ROS | 3.810 ms deadline miss, 262.5 Hz 미달 |
| `CONTACT` | detector | false positive/negative, 해제 지연 |
| `FEEDBACK` | leader 반력 | 방향 반대, 진동, pose jump, clip 지속 |
| `ROBOT` | driver/controller | impedance controller fault, joint stale |
| `OPS` | 운용 절차 | 잘못된 payload ID, 접촉 중 zero-set |

## 상태

- `OPEN`: 원인 미확정 또는 수정 전
- `MITIGATED`: 임시 회피 적용
- `VERIFY`: 수정 후 재실험 대기
- `CLOSED`: 같은 조건의 재실험에서 해결 확인
- `WONT_FIX`: 범위 밖이며 이유를 기록함

## 실패 목록

| ID | 날짜 | 분류 | 단계 | 증상 | 상태 | 상세 항목 |
|---|---|---|---|---|---|---|
| - | - | - | - | 아직 기록된 실기 실패 없음 | - | - |

새 실패를 기록할 때 위 행을 지우고 `FT-YYYYMMDD-NN` 형식으로 추가한다.

## 상세 기록 템플릿

```markdown
## FT-YYYYMMDD-NN: 짧은 제목

- 상태: OPEN
- 분류: SENSOR/ZERO/SYNC/DATA/MODEL/RUNTIME/CONTACT/FEEDBACK/ROBOT/OPS
- 누가: 운용자, 사용한 로봇/팔
- 언제: 날짜·시간, 전원 ON 후 경과 시간
- 어디서: SBC/PC, node, topic, 파일 경로
- 무엇을: 기대값과 실제 증상
- 왜: 현재 원인 가설과 근거
- 어떻게: 재현 순서

### 실행 조건

- git commit 또는 source hash:
- model.ts / metadata SHA-256:
- zero_set_id:
- payload_id:
- controller_config_hash:
- sensor/tool 장착 상태:
- feedback stage: OFF/40%/100%
- 관련 명령:

### 정량 결과

- force max/p95/RMSE:
- false contact activation:
- contact miss:
- inference p99/max:
- source age / record gap:
- max feedback torque:
- max pose step:
- velocity reversal rate:

### Artifact

- raw NPZ/JSON:
- Leader CSV:
- analyzer report:
- authorization:
- terminal/ROS log:

### 원인 분석

관찰과 추측을 분리해서 작성한다.

### 조치

- 수정 내용:
- 영향 범위:
- rollback 방법:

### 재검증

- 동일 조건 재실험 결과:
- 다른 zero-set/time/power-cycle 재실험 결과:
- 최종 상태:
```

## 기록 규칙

1. raw artifact는 수정하지 않는다.
2. 실패한 모델과 report도 삭제하지 않는다.
3. threshold를 바꿔 통과시킨 경우 기존 threshold 실패를 별도 기록한다.
4. contact가 섞인 free-space episode는 수정하여 재사용하지 않고 폐기 표시한다.
5. 원인이 여러 개면 대표 ID 아래에 가설별 증거를 나눈다.
