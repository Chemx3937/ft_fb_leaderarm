# 모방학습 episode의 contact observer 검증

## 결론

**실제 물리 접촉 정답과의 정확도: 계산 불가(NOT EVALUABLE).**
102개 episode를 확인했지만 독립적인 same-clock contact interval, 수동 annotation 또는
외부 접촉 센서 label이 있는 episode는
`0/102`개다.
반면 `102`개 모두 저장 state의 source가
`/contact_observer/right/observation`이므로 이를 실제 정답이라고 간주하면 순환 검증이 된다.

아래 수치는 실제 정답 정확도가 아니라 기존 IL observer 출력과의 **참고용 일치도**다.
현재 free-space 모델 residual에 현행 Schmitt detector(`ON 2.0 N / OFF 1.2 N`,
`8/20 ms` hold)를 적용하고, 모방학습 data에 저장된 기존 contact state와 비교했다.
전체 sample precision/recall/F1/accuracy는 `0.773/`
`0.699/0.734/0.864`이며
balanced accuracy는 `0.812`, false CONTACT activation은
`644`회다.

**이 수치는 물리 contact 정확도 판정이 아니라 IL 입력 contact channel과의 일치도다.**
저장 state도 이전 free-space model이 만든 값이고 독립 접촉 label이나 같은 시계의 수동
contact interval이 없다. 따라서 precision/recall이 높더라도 정식 `CO-04` PASS로
판정할 수 없다.

| 입력 범위 | samples | precision | recall | F1 | accuracy | balanced accuracy | false activations |
|---|---:|---:|---:|---:|---:|---:|---:|
| 전체 | 465,722 | 0.773 | 0.699 | 0.734 | 0.864 | 0.812 | 644 |
| 262.5 Hz q,dq | 214,925 | 0.744 | 0.690 | 0.716 | 0.860 | 0.804 | 271 |
| 30 Hz joint interpolation | 250,797 | 0.797 | 0.706 | 0.749 | 0.867 | 0.818 | 373 |

Reference event `312`개 중 current replay와 겹친 event는
`303`개(event recall `0.971`)다.
일치 event의 onset latency p50/p95/max는
`0.000/18.828/`
`1146.229 ms`다.

## 산출물

- [전체 요약 plot](summary.png)
- [102개 episode별 plot PDF](episode_plots.pdf): PDF page `n+1`이 `episode_nnn`
- [episode별 수치 CSV](episode_metrics.csv)
- [기계 판독용 전체 report](analysis.json)

각 episode plot은 current residual force norm, ON/OFF threshold, 저장 reference state와
current replay state를 함께 표시한다. 붉은 영역은 두 state의 불일치 구간이다.

## 입력과 제한

- 모델 SHA-256: `8c61261bb2fdd0151291f9c52ca627e59e04d71bc5655c92df5081943280ee8b` (`approved=false` diagnostic bundle)
- 262.5 Hz `q,dq` 직접 replay: `46`개
- 30 Hz joint 보간 replay: `56`개
- episode별 detector는 저장된 약 1초 pre-roll 시작에서 reset했다. 정식 runtime의
  episode 사이 연속 state와는 다르며 독립 episode 비교를 위한 계약이다.
- 동일 source identity 중복 `2`개는 첫 valid row만
  사용했다.
- 물리 정확도 확정에는 동일 시계의 독립 contact interval annotation이 필요하다.

## 육하원칙

- 누가/언제: 이 offline 분석기가 2026-08-26T14:17:27.391018+00:00에 저장 data를 replay했다.
- 어디서/무엇을: `/data/logistic_box_contact_observer`의 102개 episode에서 current contact state를 계산했다.
- 어떻게: `W_contact = W_raw - W_free_hat`의 force norm에 현행 Schmitt detector를 적용하고
  저장된 IL contact channel과 sample/event 단위로 비교했다.
- 왜: 새 model/observer가 기존 모방학습 observation과 얼마나 호환되는지 확인하기 위해서다.
