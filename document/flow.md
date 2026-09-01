# Physical FT free-space wrench 전체 흐름

## 한눈에 보기

```text
안전/계약 확정
  → FT sensor 특성 확인
  → 독립 zero-set별 무접촉 데이터 수집
  → dataset 검증
  → 모델 학습/비교 또는 현재 운용 모델 선택
  → 운용 허용 모델 observer-only 검증
  → feedback OFF 분석
  → 40% 승인/검증
  → 100% 승인/검증
  → IL 데이터 수집 적용
```

## 단계별 문서

| 단계 | 할 일 | 완료 조건 | 관련 문서 |
|---:|---|---|---|
| 0 | 목표, payload, frame, 안전 한계 확정 | 변경 불가능한 운용 계약 작성 | [TODO](TODO_LIST.md), [패키지 비교](<compare ft_fb_leaderarm and fb_leader arm.md>) |
| 1 | AFT drift/noise/zero 반복성 측정 | 1 N error budget에 충분한 sensor 여유 | [FT 점검표](FTsensor_check_list.md) |
| 2 | 무접촉 episode 수집 | 최소 3개, 권장 8~10개 독립 zero group | [명령어](command.md) |
| 3 | dataset 검증 | 모든 episode 계약 통과, group split 생성 | [명령어](command.md), [TODO](TODO_LIST.md) |
| 4 | 모델 선택 | 현재 고정 SHA 쌍 또는 정식 robust gate 통과 모델 확정 | [현재 architecture](free_space_wrench_model_architecture.md), [pseudocode](free_space_wrench_model_pseudocode.md) |
| 5 | observer-only 실기 | 262.5 Hz, FREE p95/p99 1 N, hard max 2 N, false contact 0 | [명령어](command.md) |
| 6 | feedback OFF evidence | FREE 3회+CONTACT, analyzer `GO` | [명령어](command.md) |
| 7 | 40% feedback | 방향·진동·pose jump 정상, analyzer `GO` | [명령어](command.md) |
| 8 | 100% feedback | authorization chain 검증과 제한 운용 통과 | [명령어](command.md) |
| 9 | IL recorder 적용 | source/frame/model identity가 episode에 보존 | [TODO](TODO_LIST.md) |
| 모든 단계 | 실패와 변경 기록 | artifact와 육하원칙이 연결됨 | [문제 기록](problem/README.md) |

## 중단 조건

다음 중 하나라도 발생하면 다음 단계로 진행하지 않는다.

- 접촉이 free-space 학습 episode에 포함됨
- 다른 payload/tool/controller 계약이 같은 dataset에 섞임
- FT drift/noise만으로 1 N에 접근함
- 새 모델을 정식 승격할 때 validation 또는 held-out test가 p99 1 N, group p95 1 N,
  hard max 2 N 중 하나를 넘음
- inference p99가 3.048 ms 또는 hard max가 3.810 ms를 넘음
- observer invalid/stale/deadline miss가 발생함
- FREE false contact가 한 번이라도 발생함
- feedback 방향 반대, 진동, pose jump, clip 지속이 관찰됨
- analyzer 또는 authorization 결과가 `NO-GO`

실패 시 [문제 기록](problem/README.md)에 새 문제 파일을 추가하고 같은 단계에서 원인 제거 후
독립 조건으로 재검증한다.
