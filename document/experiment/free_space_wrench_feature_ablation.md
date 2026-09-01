# Free-space wrench feature·ensemble·data-size ablation

> 이 문서는 현재 모델을 선택하기 전의 2026-08 비교 실험 기록이다. 현재 운용
> architecture는 [별도 문서](../free_space_wrench_model_architecture.md)를 따른다.

- 수행일: 2026-08-19
- 대상 dataset: `right_final13_20260819`
- 범위: 저장된 train/validation data만 사용하는 offline 진단
- hardware: robot/AFT/hand driver 미사용, 이동·zero-set 없음
- 관련 문제: [FT-20260819-01](../problem/FT-20260819-01.md)

## 목적

기존 5개 모델 외에 causal acceleration smoothing과 더 짧은 history가 동적 residual을
줄이는지 확인한다. 또한 모델 평균 ensemble과 train zero-set group 수 증가가
validation 오차에 미치는 영향을 분리한다.

이미 공개된 held-out test 3개는 새 후보 선택이나 이 분석에 사용하지 않았다.

## 방법

공통 조건은 기존 final13 학습과 같다.

- split seed: `7`
- train/validation zero-set groups: `7/3`
- validation groups: `div02`, `div05`, `div07`
- epochs: 최대 `60`
- batch: `1024`
- learning rate: `0.001`
- group당 최대 train windows: `20,000`
- checkpoint 선택: validation force RMSE
- 최종 비교 순서: validation force max, p95, RMSE

추가 후보는 다음 두 개다.

| 후보 | 입력 | 의도 |
|---|---|---|
| `smoothed_dynamic_mlp` | 최근 8 sample qdd 평균과 현재 q/dq | raw finite-difference qdd의 순간 변동 완화 |
| `short_history_mlp` | 최근 8 sample의 q/dq/qdd | 기존 16-sample history보다 짧은 약 30.5 ms 동역학 비교 |

Smoothing은 현재와 과거 7개 sample만 사용하는 causal 연산이다. 실패한 진단 후보를
runtime 계약에 추가하지 않기 위해 package source는 수정하지 않고 보존된 standalone
스크립트에서 기존 `train_ablation` 함수를 재사용했다.

## 실행·artifact

```bash
source /home/vision/venv_act/bin/activate
cd /home/vision/dualarm_ws/src/ft_fb_leaderarm
PYTHONPATH=. python \
  /home/vision/.ros/ft_fb_leaderarm/models/right_final13_feature_ablation_20260819/run_final13_feature_ablation.py

python3 -m json.tool \
  /home/vision/.ros/ft_fb_leaderarm/models/right_final13_feature_ablation_20260819/validation_report.json
```

스크립트 기본 output은 `/tmp/right_final13_feature_ablation_validation_20260819.json`이며,
검토한 report를 아래 경로에 보존했다.

- report:
  `/home/vision/.ros/ft_fb_leaderarm/models/right_final13_feature_ablation_20260819/validation_report.json`
- report SHA-256:
  `8430de6b94310b9831ad9cfbd0eba211f0fd4164329028a96a84d70cdc0a743c`
- script SHA-256:
  `5d4ba6198b633f0859b421e1d6f847d20a4150aed34fba9ff366f766f9d8c7bd`
- 실행시간: 약 `42.5초`, CPU

## 단일 모델 결과

모든 값은 같은 diverse validation 3 groups에 대한 결과다.

| 모델 | force max [N] | p95 [N] | RMSE [N] | 판정 |
|---|---:|---:|---:|---|
| static linear | 5.793 | 2.565 | 1.257 | FAIL |
| dynamic MLP | 3.980 | 1.687 | 0.951 | FAIL |
| history MLP, 16 samples | 4.409 | 1.696 | 0.912 | FAIL |
| history LSTM, 16 samples | 4.871 | 1.624 | 0.882 | FAIL |
| history GRU, 16 samples | 5.054 | 1.661 | 0.895 | FAIL |
| qdd-smoothed dynamic MLP, 8 samples | 4.001 | **1.561** | 0.879 | FAIL |
| short-history MLP, 8 samples | **3.947** | 1.668 | **0.872** | FAIL |

새 단일 모델 중 qdd smoothing은 p95를, short history는 max와 RMSE를 일부 줄였다.
그러나 최선 max가 `3.947 N`이므로 둘 다 `1 N` gate와 차이가 크다. 단순히 history를
짧게 하거나 qdd를 평균하는 한 가지 변경만으로는 해결되지 않았다.

같은 실행에서 측정한 model-only 처리율은 다음과 같다.

| 모델 | p99 등가 처리율 [Hz] | 최악 등가 처리율 [Hz] | 요구값 |
|---|---:|---:|---:|
| dynamic MLP | 68,100 | 35,200 | 262.5 Hz PASS |
| qdd-smoothed dynamic MLP | 36,100 | 22,900 | 262.5 Hz PASS |
| short-history MLP | 39,600 | 27,900 | 262.5 Hz PASS |

CPU 부하에 따라 이전 benchmark와 수치는 달라질 수 있지만 세 모델 모두 요구값과의
차이가 매우 크다. 이는 ROS를 제외한 model-only 값이다.

새 모델의 validation group별 결과는 다음과 같다.

| 모델 | group | max [N] | p95 [N] | RMSE [N] |
|---|---|---:|---:|---:|
| qdd-smoothed MLP | `div02` | 4.001 | 2.078 | 1.117 |
| qdd-smoothed MLP | `div05` | 3.167 | 1.372 | 0.877 |
| qdd-smoothed MLP | `div07` | 2.176 | 1.083 | 0.638 |
| short-history MLP | `div02` | 3.947 | 2.374 | 1.234 |
| short-history MLP | `div05` | 2.578 | 1.263 | 0.804 |
| short-history MLP | `div07` | 1.723 | 0.895 | 0.529 |

두 후보 모두 `div02`의 near/far·J2/J3 동작에서 가장 어렵다. 데이터나 모델을 추가할
때 이 동작을 우선해야 한다.

## 단순 평균 ensemble 결과

각 모델의 6축 출력을 같은 비율로 평균했다. 별도 weight tuning은 하지 않았다.

| 구성 | residual 상관 | max [N] | p95 [N] | RMSE [N] | p99/최악 처리율 [Hz] |
|---|---:|---:|---:|---:|---:|
| dynamic + qdd-smoothed | 0.869 | 3.884 | 1.561 | 0.884 | 19,900 / 7,200 |
| dynamic + short-history | **0.726** | **3.530** | 1.541 | 0.847 | **21,300 / 15,600** |
| qdd-smoothed + short-history | 0.846 | 3.555 | 1.540 | **0.841** | 16,900 / 13,200 |
| 3개 평균 | 0.726~0.869 | 3.581 | **1.522** | 0.842 | 12,900 / 10,900 |

max 기준 최선은 `dynamic + short-history`로, dynamic 단독보다 max/p95/RMSE가
약 `11.3/8.6/11.0%` 감소했다. 하지만 max `3.530 N`으로 여전히 실패다. 모델들의
force residual 상관도 `0.726~0.869`로 높아 공통 오차가 많이 남는다. 따라서
ensemble은 계산 속도 문제가 아니라 정확도 개선 폭이 부족해 현재 채택하지 않는다.

표의 Hz는 두세 모델을 CPU에서 순차 실행하고 평균까지 포함해 직접 측정한 model-only
등가 처리율이다. 모두 요구값 `262.5 Hz`보다 충분히 빠르지만 실제 ROS publish rate는
아니다.

## 데이터 크기 learning curve

동일 `dynamic_mlp`와 동일 validation 3 groups를 유지하고 train group만 중첩해서
늘렸다.

| train groups | train windows | 추가된 범위 | validation max [N] | p95 [N] | RMSE [N] |
|---:|---:|---|---:|---:|---:|
| 3 | 15,471 | task 3개만 | 11.168 | 6.747 | 3.027 |
| 5 | 52,468 | task 3개 + `div03/div04` | 4.703 | 1.946 | 1.075 |
| 7 | 92,468 | 위 범위 + `div06/div09` | 3.980 | 1.687 | 0.951 |

3→5→7 groups에서 모든 지표가 일관되게 감소했다. 특히 task-only에서 diverse train을
추가했을 때 개선이 크므로 데이터 확대는 효과가 있다. 다만 5→7 groups의 개선은
max/p95/RMSE 기준 약 `15.4/13.3/11.5%`이고 최종 max가 여전히 `3.980 N`이다.

이 실험은 sample 수뿐 아니라 새로운 궤적 종류도 동시에 늘렸으므로 “같은 데이터를
길게 반복하면 좋아진다”는 뜻이 아니다. 현재 evidence가 지지하는 것은 독립 zero-set과
새 상태 범위를 포함하는 다양한 데이터가 도움이 된다는 것이다.

## 데이터 추가에 대한 판단

데이터를 더 모으면 개선될 가능성은 높지만, 데이터만으로 `1 N`을 달성한다고 보장할
수는 없다. 다음 수집은 같은 task 반복이 아니라 현재 train에서 빠지고 validation에서
오차가 컸던 범위를 겨냥해야 한다.

우선 추가할 최소 batch는 독립 hardware zero-set 6 groups다.

- near/far·J2/J3 (`div02` 유사) 2 groups
- Cartesian X/Z (`div05/div07` 유사) 각 1 group
- 3D 곡선과 빠르되 안전한 가감속 2 groups

기존 validation 3 groups는 그대로 보존하고 새 6 groups만 train에 추가한다. 새
ablation을 validation으로 확정하기 전에는 최종 test를 수집하거나 열지 않는다.
validation 방법을 고정한 뒤에는 다른 날짜·전원 cycle·독립 zero-set에서 최소 3개를
새 held-out test로 수집해야 한다.

새 6 groups 후에도 validation max가 `3 N` 부근에서 정체되면 데이터 수집만 계속하지
않고 sensor/robot-state 동기, payload·중력 모델과 target noise 한계를 다시 분리한다.

## 결론

- 다른 단일 모델 7개 중 `1 N`을 통과한 모델은 없다.
- 단순 ensemble은 max를 `3.530 N`까지 낮췄지만 해결책은 아니다.
- 추가 데이터는 명확히 도움이 됐으나 “개수”보다 누락된 동적 상태의 coverage가
  중요하다.
- 다음 단계는 targeted train 6 groups 수집이다. 이는 robot 이동·zero-set·수집이
  필요한 hardware 단계이므로 사용자 승인과 새 session의 warm-up 확인 후 수행한다.

다음 장비 session의 split 보호, episode별 역할과 재개 절차는
[FT-20260819-01 재개 체크포인트](../problem/FT-20260819-01.md#다음-session-재개-체크포인트)를
따른다.
