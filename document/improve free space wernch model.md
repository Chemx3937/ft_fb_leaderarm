# Free-space wrench model 개선 캠페인

## 목표와 판정 계약

- 목표: 오른팔 무접촉 AFT wrench의 force-vector 예측 오차를 줄인다.
- 합격 기준: development validation과 새 held-out test에서 각각
  `max(||Fmeasured-Fpredicted||) <= 1 N`이다.
- 현재 비교 데이터: 고정 train 13 zero groups와 development validation
  `div02/div05/div07`만 사용한다.
- 이미 공개된 과거 held-out `div01/div08/div10`은 어떤 후보에도 다시 사용하지 않는다.
- 여러 후보를 같은 validation으로 비교하므로 여기서 고른 모델은 **offline 최선 후보**일
  뿐이다. 배포 승격에는 방법을 고정한 뒤 새 독립 held-out test가 필요하다.
- hardware, ROS graph, observer/runtime 설정과 acceptance contract는 별도 승인 전까지
  변경하지 않는다.

현재 기준 모델은 train-only payload gravity를 먼저 빼고 causal qdd 8-sample 평균을
사용한 residual MLP다. seed 8 validation max/p95/RMSE는
`2.955/1.267/0.725 N`이며 미승인이다.

## 공통 평가 순서

1. 데이터·계약 누수 guard와 runnable self-check를 통과한다.
2. 한 번에 한 방법만 기준 모델에 적용해 효과를 분리한다.
3. 공통 validation3에서 force max, p95, RMSE와 group별 max를 기록한다.
4. 개선 후보는 여러 seed와 causal/runtime 가능성을 재검증한다.
5. strict max가 `1 N`을 넘으면 runtime bundle을 만들지 않는다.

## 적용 방법 목록

| 순서 | 방법 | 핵심 설명 | 상태 |
|---:|---|---|---|
| 0 | 기본 5개 모델 | static/dynamic/history MLP, LSTM, GRU를 비교했다. | 완료·실패 |
| 1 | 데이터 확대 | train 7→13 groups와 targeted 동작 coverage를 늘렸다. | 완료·개선, 실패 |
| 2 | qdd smoothing·짧은 history | causal qdd 평균과 8/16-sample 이력을 사용했다. | 완료·개선, 실패 |
| 3 | 단순 ensemble | 서로 다른 모델의 예측을 평균했다. | 완료·개선, 실패 |
| 4 | payload gravity + residual | train-only 질량·CoM 중력을 빼고 동적 residual만 학습했다. | 완료·현재 최선, 실패 |
| 5 | timestamp lag | 사후 lag와 causal lag feature를 비교했다. | 완료·실패 |
| 6 | causal output filter | median과 EMA 20/10/5/2.5 Hz를 지연과 함께 평가했다. | 완료·실패 |
| 7 | pose/history KNN correction | train residual을 posture/dynamic/slow-history 이웃으로 보정했다. | 완료·실패 |
| 8 | target05/06 제외 | 고가감속 두 group이 성능을 해치는지 제거 재학습했다. | 완료·제거 시 악화 |
| 9 | zero median 직접 분해 | 수집 metadata의 기준 자세 force median을 session bias로 직접 분리한다. | 완료·실패 |
| 10 | zero median 학습 보정 | train group만으로 zero median→session residual affine 관계를 학습한다. | 완료·typical 개선, max 실패 |
| 11 | session-balanced sampling | 긴 episode가 학습을 지배하지 않도록 group별 표본 수를 맞춘다. | 완료·max 악화 |
| 12 | motion-balanced sampling | 속도·가속도 구간별 표본/손실 가중치를 균형화한다. | 완료·악화 |
| 13 | robust loss | Huber/MAE 혼합으로 순간 target noise의 학습 영향만 줄인다. | 완료·효과 없음 |
| 14 | force-tail loss | moment보다 force와 큰 force residual에 학습 예산을 집중한다. | 완료·효과 없음 |
| 15 | regularized linear residual | 확장된 causal feature에 ridge를 적용해 과적합 여부를 분리한다. | 완료·최선 |
| 16 | MLP capacity | 작은/큰 MLP를 비교해 underfit과 overfit을 분리한다. | 완료·악화 |
| 17 | multi-scale causal history | 여러 시간폭의 dq/qdd 평균으로 최근 이동 이력을 표현한다. | 완료·ridge와 결합해 최선 |
| 18 | recurrent residual | physical residual에 32-sample GRU history를 적용한다. | 완료·typical 개선, max 실패 |
| 19 | motion-regime experts | 저속/고속 residual 모델을 causal acceleration으로 부드럽게 혼합한다. | 완료·악화 |
| 20 | multi-seed ensemble | 최선 후보의 seed별 예측을 평균해 분산을 줄인다. | 완료·추가 효과 없음 |
| 21 | 최선 후보 causal filter | 최선 ridge residual에 기존 median/EMA를 다시 적용한다. | 완료·strict max 실패 |
| 22 | 단순 비선형 ridge | elementwise quadratic과 random Fourier feature를 추가한다. | 완료·유의미한 추가 효과 없음 |
| 23 | train-group 교차검증 | train13 내부 leave-one-zero-group-out으로 방향성을 확인한다. | 완료·ridge history 우세 |

## 현재 데이터로 정당하게 평가할 수 없는 방법

| 방법 | 현재 막힌 이유 | 재개 조건 |
|---|---|---|
| 센서 온도 보정 | episode에 AFT 내부 온도가 없다. | 같은 clock의 temperature 기록 |
| 정확한 time-since-zero/warm-up 모델 | 수집 시작 시각은 있지만 hardware zero 명령 시각이 없다. | zero timestamp metadata 추가 |
| cable force 입력 | cable 장력 센서나 정량 label이 없다. | strain/force 측정 또는 고정 fixture |
| 정확한 rigid-body inverse dynamics | adapter 질량·CoM과 AFT 원점 이동이 미확정이다. | 실측값과 수정된 URDF |
| 새 held-out 최종 승인 | 현재 방법이 아직 고정되지 않았다. | offline 최선 후보 고정 후 새 수집 승인 |
| tree/boosting 계열 | 현재 환경에 `scikit-learn`이 없고 새 의존성을 정당화할 개선 근거가 없다. | 선정 ridge가 새 train에서도 plateau일 때 |

## 결과 기록

seed 7/8/9 평균이며 과거 held-out은 사용하지 않았다.

| 방법 | validation max [N] | p95 [N] | RMSE [N] | 판정 |
|---|---:|---:|---:|---|
| 기준 physical residual MLP | **2.931** | **1.277** | `0.734` | FAIL |
| target05 제외 | `3.018` | `1.327` | `0.769` | 악화 |
| target06 제외 | `3.013` | `1.288` | `0.741` | 악화 |
| target05/06 제외 | `3.173` | `1.350` | `0.782` | 악화 |
| zero median 직접 분해 | `2.971` | `1.319` | **0.730** | max/p95 악화 |

target05/06은 기존 train보다 가감속이 크지만 제거 시 validation이 악화됐다. 또한
hardware zero 뒤 기록된 기준 자세 force median을 그대로 빼는 방법은 RMSE만
`0.004 N` 줄이고 max/p95를 악화시켰다. 두 방법 모두 폐기하고 다음 방법으로 간다.

### 2026-08-22 전체 방법 screen

- 실행 script: `scripts/run_free_space_improvement_sweep.py`
- 최종 report:
  `/home/vision/.ros/ft_fb_leaderarm/models/right_train13_improvement_sweep_v2_20260822/improvement_sweep_report.json`
- split manifest:
  `/home/vision/.ros/ft_fb_leaderarm/models/right_train13_improvement_sweep_v2_20260822/split_manifest.json`
- SHA-256: script
  `d4f2338b0a1020119bfc379e868b4bf2deeaaa3a81dd5b6ce90cce8066e5461a`, report
  `72c9b9621ef792a8ac20b4058a7b8f5741e56347df920ac006dcc9d906d42c2e`, manifest
  `be7f8cfb440bf1fec992c0785e5a7a2b86611c98128809563ad9d1ea63bba8da`
- 범위: train13 학습과 development validation3만 사용했다. 과거 held-out/test,
  hardware, ROS graph, runtime과 acceptance contract는 사용하거나 변경하지 않았다.
- 아래 screen은 별도 표기가 없으면 사전 고정 seed 8과 공통 128-sample validation
  시작점을 사용한다.

| 방법 | max [N] | p95 [N] | RMSE [N] | 핵심 판정 |
|---|---:|---:|---:|---|
| 기준 physical residual MLP | `2.955` | `1.268` | `0.726` | 기준 재현 |
| zero median 입력 feature | `3.211` | `1.267` | `0.705` | max 악화 |
| train-only zero affine + MLP | `2.954` | `1.259` | `0.701` | max 효과 없음 |
| session-weighted sampling | `3.159` | `1.267` | `0.721` | max 악화 |
| motion-weighted sampling | `3.325` | `1.359` | `0.769` | 전부 악화 |
| Huber loss | `2.946` | `1.269` | `0.727` | 기준과 동일 수준 |
| MAE/MSE loss | `2.963` | `1.274` | `0.730` | 악화 |
| force-tail loss | `2.951` | `1.267` | `0.726` | 기준과 동일 수준 |
| 작은 MLP | `3.035` | `1.255` | `0.718` | typical만 개선 |
| 큰 MLP | `3.334` | `1.329` | `0.746` | 과적합 방향 |
| elapsed-time feature | `2.997` | `1.277` | `0.731` | 효과 없음 |
| 128-sample multi-scale MLP | `3.315` | `1.264` | `0.727` | 비선형 모델 악화 |
| base feature ridge | `2.873` | `1.285` | `0.743` | max만 소폭 개선 |
| 128-sample multi-scale ridge | `2.220` | `1.150` | `0.678` | 큰 개선 |
| 32-sample GRU | `2.889` | `1.125` | `0.657` | typical 개선, max 실패 |
| motion-regime experts | `2.992` | `1.269` | `0.729` | 악화 |

MLP 크기, robust loss와 sampling보다 선형 ridge가 더 좋았다. ridge feature 제거
실험에서는 q 변화량과 단순 방향 부호를 빼도 성능이 유지됐지만 dq 평균을 빼면 max가
`2.311 N`, qdd 평균을 빼면 `3.006 N`으로 악화됐다. 핵심 정보는 여러 시간폭의
dq/qdd 평균이다.

### 최종 offline 후보

최소 feature는 다음 54D causal vector다.

```text
sin(q)[6], cos(q)[6], current dq[6],
mean dq over 8/16/32 samples[18],
mean qdd over 8/16/32 samples[18]
```

262.5 Hz에서 최대 history는 32 samples, 약 `121.9 ms`다. train feature/target을
표준화하고 L2 regularization `1.0`인 다중출력 ridge를 physical gravity residual에
적합한다. q 변화량, direction sign, episode elapsed time과 별도 비선형 층은 쓰지 않는다.

| 평가 | max [N] | p95 [N] | RMSE [N] |
|---|---:|---:|---:|
| 기준 MLP, seed 8 | `2.955` | `1.268` | `0.726` |
| 32-sample ridge, seed 7 | `2.172` | `1.135` | `0.666` |
| 32-sample ridge, seed 8 | **`2.162`** | **`1.135`** | **`0.666`** |
| 32-sample ridge, seed 9 | `2.178` | `1.134` | `0.666` |
| seed 평균 ensemble | `2.171` | `1.135` | `0.666` |
| zero affine + ridge, seed 8 | `2.175` | `1.066` | `0.609` |

seed 8 group별 max는 div02 `2.145 N`, div05 `2.162 N`, div07 `2.070 N`이다.
기준 대비 max는 약 `26.9%`, p95는 `10.5%`, RMSE는 `8.3%` 줄었다. zero-affine
결합은 p95/RMSE를 더 줄이지만 max 우선 선택에서 단독 ridge보다 나쁘고 software
bias 계약 변경도 필요하므로 채택하지 않는다. seed ensemble도 이득이 없어 쓰지 않는다.

train13 내부 13-fold leave-one-zero-group-out에서도 base ridge 대비 32-sample ridge가
p95 `1.953→1.655 N`, RMSE `1.057→0.925 N`, group max 중앙값
`3.215→2.663 N`으로 개선됐다. 두 후보 모두 train-fold 절대 gate는 실패했지만,
multi-scale 방향이 validation3에만 우연히 맞은 것은 아니라는 근거다.

elementwise quadratic ridge는 max `2.309 N`으로 악화됐다. 64개 random Fourier
feature를 더한 후보의 최선 max는 `2.161 N`으로 선형 ridge와 `0.001 N` 이내이고
p95/RMSE 개선도 일관되지 않아 추가 복잡도를 채택하지 않는다.

### 성능이 좋았던 모델 방법 5개

아래는 filter, 같은 방법의 seed·regularization 중복을 제외한 모델 자체 비교다.
현재 계약대로 validation의 force-vector max 오차를 우선해 정리했다.

| 순위 | 방법 | max [N] | p95/RMSE [N] | 쉽게 말하면 | 채택 판단 |
|---:|---|---:|---:|---|---|
| 1 | random Fourier feature + ridge | `2.161` | 일관된 개선 없음 | 같은 입력을 64개 비선형 파형으로 펼친 뒤 ridge를 적용한다. | `0.001 N` 미만 차이라 복잡도만 늘어 폐기 |
| 2 | 32-sample multi-scale ridge | `2.162` | `1.135/0.666` | 최근 `122 ms` 동안의 관절 속도·가속도 평균으로 현재 동적 오차를 선형 보정한다. | **최종 offline 후보** |
| 3 | zero affine + 32-sample ridge | `2.175` | `1.066/0.609` | zero 자세의 force median으로 session별 편향을 먼저 보정한 뒤 ridge를 적용한다. | 보통 오차는 최저지만 max가 나쁘고 bias 계약 변경이 필요해 폐기 |
| 4 | 128-sample multi-scale ridge | `2.220` | `1.150/0.678` | 약 `488 ms`의 더 긴 움직임 이력까지 ridge에 넣는다. | 짧은 32-sample 이력보다 느린 과거가 오히려 방해해 폐기 |
| 5 | elementwise quadratic ridge | `2.309` | 일관된 개선 없음 | 각 입력의 제곱항을 더해 단순한 비선형 관계를 표현한다. | 선형 32-sample ridge보다 나빠 폐기 |

random Fourier 후보의 max는 반올림 전에도 최종 ridge와 약 `0.0004 N` 차이뿐이다.
이는 현재 센서 noise보다 훨씬 작고 p95/RMSE도 좋아지지 않았다. 따라서 정확도는
동률로 보고, feature가 54개뿐이며 계산·검증이 단순한 32-sample 선형 ridge를 골랐다.

### 최종 후보의 causal filter 재평가

| filter | max [N] | p95 [N] | p99 [N] | RMSE [N] | `>1 N` 최장 |
|---|---:|---:|---:|---:|---:|
| raw ridge residual | `2.162` | `1.135` | `1.435` | `0.666` | `209.5 ms` |
| EMA 20 Hz | `1.850` | `0.966` | `1.241` | `0.566` | `483.8 ms` |
| EMA 10 Hz | `1.665` | `0.882` | `1.140` | `0.517` | `510.5 ms` |
| EMA 5 Hz | `1.544` | `0.807` | `1.045` | `0.475` | `521.9 ms` |
| EMA 2.5 Hz | `1.444` | `0.758` | `0.981` | `0.447` | `548.6 ms` |

모든 filter에서 기존 `2 N/8 ms` detector false-contact는 0회였지만 strict max는
실패했다. 낮은 cutoff는 p99를 줄이는 대신 `>1 N` 지속시간과 contact 지연을 늘린다.
따라서 filter도 runtime에 채택하지 않는다.

### 최종 후보의 inference 주기 확인 방법

현재 확정된 것은 **설계 호출 주기**다. 수집·학습·observer 계약이 `262.5 Hz`이므로
새 sample마다 한 번 추론하며 목표 period는 `1000 / 262.5 = 3.8095 ms`다. 처음
32 samples, 약 `121.9 ms`는 causal history를 채우는 warm-up이고, 이후에는
32 samples마다 한 번이 아니라 **매 sample마다** 예측한다.

2026-08-22에 정확히 같은 54D 전처리, TorchScript ridge residual과 Pinocchio payload
gravity를 포함하는 diagnostic bundle을 만들고 target PC에서 2,000회 측정했다.

| 항목 | 결과 | 기준 | 판정 |
|---|---:|---:|---|
| mean latency | `0.05463 ms` | 참고 | 약 `18,304 Hz` |
| p99 latency | `0.08783 ms` | `<=3.04762 ms` | PASS |
| max latency | `0.60003 ms` | `<=3.80952 ms` | PASS |

- bundle/report:
  `/home/vision/.ros/ft_fb_leaderarm/models/right_train13_ridge_short_multiscale_bundle_v3_20260822`
- runtime/offline 전체 validation prediction 최대 차이: `0.000206 N`
- runtime 경로 validation max/p95/RMSE: `2.162/1.135/0.666 N`
- model-only `FS-04` timing gate: PASS
- accuracy `FS-03`: FAIL이므로 metadata는 `approved=false`이고 observer는 이 bundle을
  의도대로 거부한다.

남은 실측은 ROS 유효 publish rate다. 정확도 gate를 통과해 bundle이 승인된 뒤 같은
bundle을 observer에 연결하고 아래 passive evaluator로 10초 측정한다.
`valid_publish_hz >= 262.5`, deadline miss·invalid·stale가 모두 0이면 `FS-05` PASS다.

```bash
ros2 run ft_fb_leaderarm ft_observer_runtime_evaluate -- \
  --duration-s 10 \
  --output /tmp/right_ridge_observer_runtime.json
```

이 명령은 현재 상태에서 실행하지 않는다. runtime-compatible predictor는 구현됐지만
`max<=1 N`을 실패한 bundle의 승인 상태를 바꾸거나 observer 설정에 연결해서는 안 된다.
단순 `ros2 topic hz`는 전체 publish 수만 보여 주며 valid prediction, deadline miss와
stale을 구분하지 못하므로 최종 판정에는 위 evaluator를 사용한다.

## 최종 선택

- 현재 offline 최선: `physical gravity + 32-sample multi-scale ridge residual`
- validation max/p95/RMSE: `2.162/1.135/0.666 N`
- strict `1 N` 합격 모델: 없음
- runtime 승인 모델: 없음
- 다음 단계: 현재 데이터로 검증 가능한 후보 screen은 종료한다. strict gate를 유지하면
  missing-state 계측과 새 train이 필요하고, gate를 바꾸려면 별도 승인과 contact latency
  기준 확정이 먼저다.
