# 모방학습 episode의 free-space wrench 모델 검증

## 결론

현재 diagnostic 모델을 102개 logistic-box 모방학습 episode에 offline replay했다.
주 지표인 **저장 CONTACT에서 200 ms 이상 떨어진 안정 FREE 구간**의 aggregate force
RMSE/p95/p99/max는 `1.005/`
`1.750/2.424/`
`7.396 N`이다.
현재 기준을 그대로 대입한 진단 판정은 **FAIL**이며,
episode p95 실패는 `102/102`개다. 이 결과는 contact가 없는
독립 zero-set test가 아니므로 정식 `FS-03` 승격 evidence는 아니다.

| 200 ms 안정 FREE 입력 범위 | samples | 평균 오차 [N] | <=1 N | <=2 N | <=3 N | <=4 N | 최대 [N] |
|---|---:|---:|---:|---:|---:|---:|---:|
| 전체 | 315,146 | 0.888 | 68.3% | 97.4% | 99.6% | 99.9% | 7.396 |
| 262.5 Hz q,dq | 148,483 | 0.882 | 68.7% | 97.1% | 99.5% | 99.9% | 7.396 |
| 30 Hz joint interpolation | 166,663 | 0.893 | 67.9% | 97.6% | 99.7% | 99.9% | 6.997 |

전환 주변 제외 폭에 따른 민감도는 다음과 같다. 0 ms는 저장 state가 FREE인 모든 구간이다.

| CONTACT guard [ms] | samples | 평균 오차 [N] | <=1 N | <=2 N | <=3 N | <=4 N | 최대 [N] |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 0 | 340,296 | 0.970 | 66.2% | 95.5% | 98.3% | 99.0% | 29.484 |
| 50 | 333,901 | 0.928 | 66.8% | 96.3% | 99.0% | 99.5% | 14.670 |
| 100 | 327,471 | 0.906 | 67.4% | 96.9% | 99.3% | 99.8% | 10.667 |
| 200 | 315,146 | 0.888 | 68.3% | 97.4% | 99.6% | 99.9% | 7.396 |
| 500 | 282,113 | 0.861 | 70.6% | 97.9% | 99.7% | 100.0% | 6.997 |

현재 gate: aggregate p99 `<=1 N`, episode FREE p95 `<=1 N`, hard max `<=2 N`.

## 산출물

- [전체 요약 plot](summary.png)
- [안정 비접촉 분석 plot](stable_free_summary.png)
- [102개 episode별 plot PDF](episode_plots.pdf): PDF page `n+1`이 `episode_nnn`
- [episode별 수치 CSV](episode_metrics.csv)
- [기계 판독용 전체 report](analysis.json)

각 episode plot은 sensor-frame 6축 measured/predicted wrench와 force residual을 표시한다.
붉은 음영은 저장된 기존 observer의 CONTACT 구간이며, 정확도 지표에서는 제외했다.

## 입력 재구성과 제한

- 모든 episode에 `free_space_wrench_prediction.zarr`가 없어 모델 SHA-256
  `8c61261bb2fdd0151291f9c52ca627e59e04d71bc5655c92df5081943280ee8b`의 출력을 다시 계산했다.
- `46`개는 별도 262.5 Hz observer
  log의 실제 `q,dq`를 사용했다.
- `56`개는 high-rate log가 없어
  저장된 30 Hz joint를 보간하고 `dq,qdd`를 재구성했다. 이 subset은 입력 근사 오차가
  포함되므로 별도 집계했다.
- 동일 source identity의 valid→invalid 상태 전이 `2`개는
  runtime의 duplicate-source 계약과 같이 첫 valid row만 사용했다.
- FREE/CONTACT 구분은 모방학습 data에 저장된 이전 observer state를 사용했다. 이는
  독립 ground truth가 아니다. 전환 오염을 줄이기 위해 200 ms guard를 주 지표로 삼았지만,
  잘못 저장된 장시간 FREE/CONTACT 구간 자체는 교정할 수 없다.
- model bundle은 `approved=false`인 diagnostic artifact이며 runtime 설정은 변경하지 않았다.

## 육하원칙

- 누가/언제: 이 offline 분석기가 2026-08-26T14:17:27.391018+00:00에 기존 저장 data만 읽었다.
- 어디서/무엇을: `/data/logistic_box_contact_observer`의 102개 episode에 현재 free-space model을 replay했다.
- 어떻게: episode 시작 전 pre-roll로 32-sample history를 채우고, raw FT에서 예측 wrench를
  뺀 force norm을 reference-FREE 구간에서 집계했다.
- 왜: 모방학습 domain에서 모델의 zero-set·자세·동작 일반화 성능을 확인하기 위해서다.
