# Free-space wrench 실사용 residual 비교

## 결론

동일한 stable-FREE 200 ms 구간 315,146샘플에서 current pipeline은
V7 pipeline보다 일반 오차가 작았다. RMSE는 1.990 N에서
1.005 N(-49.5%), p95는
3.094 N에서 1.750 N
(-43.4%), p99는 3.469 N에서
2.424 N(-30.1%)으로 줄었다.

반면 hard max는 4.164 N에서
7.396 N(+77.6%)으로 커졌다.
따라서 결론은 **일반 오차 개선, 극단 오차 악화**이며 단일 승자를 선언하지 않는다.

| force norm | Legacy V7 | Current | 변화율 |
|---|---:|---:|---:|
| mean | 1.855 N | 0.888 N | -52.1% |
| RMSE | 1.990 N | 1.005 N | -49.5% |
| p95 | 3.094 N | 1.750 N | -43.4% |
| p99 | 3.469 N | 2.424 N | -30.1% |
| hard max | 4.164 N | 7.396 N | +77.6% |
| within 1 N | 12.7% | 68.3% | +437.3% |
| within 2 N | 58.6% | 97.4% | +66.1% |

## 육하원칙과 비교 계약

- 누가/언제: offline 비교기가 2026-09-01T08:49:00.423721+00:00에 저장 artifact를 읽었다.
- 어디서: `/data/logistic_box_contact_observer`의 연속 102개 episode를 사용했다.
- 무엇을: 저장된 V7 `contact_wrench`와 [current replay 결과](../free_space_wrench_model_validation/README.md)의 force residual norm을 비교했다.
- 어떻게: valid/model-ready/current 32-sample warmup 조건과 legacy contact state 기준 0/50/100/200/500 ms guard를 동일하게 적용했다. primary는 200 ms다.
- 왜: 모델 구조 자체가 아니라 실제 배치된 두 pipeline의 FREE residual trade-off를 진단하기 위해서다.
- 모델 증거: observer log 13개 중 12개에서 V7 SHA와 startup residual bias 활성화를 확인했고, metadata가 없는 1개는 확인 불가로 기록했다.

V7은 `/bae_r/F_e(filtered)`를 `right_base_link`에서 예측하고 80 Hz prediction LPF와
startup residual bias를 사용한다. Current 모델은 `/aft_sensor2/wrench`를 sensor frame에서
예측한다. 회전에 불변인 force norm만 비교하며 moment와 축별 값은 비교하지 않는다.

## 산출물

- [집계 및 계약 JSON](analysis.json)
- [episode별 비교 CSV](episode_metrics.csv)
- [요약 그림](summary.png)

## 제한과 다음 실험

- FREE mask가 V7의 저장 contact state에서 만들어져 V7에 유리한 선택 편향이 있다.
- 56개 episode의 current 입력은 30 Hz joint interpolation이다.
- 서로 다른 target/frame의 실사용 residual 비교이므로 FS-03 또는 CO-04 evidence가 아니다.
- 최종 모델 교체 판단 전 별도 승인 아래 독립 zero-set 3개 이상에서 두 target을 동시에 저장하고, 동일 시간 구간의 RMSE/p95/p99/max와 FREE false CONTACT를 다시 비교한다.

## 재현

```bash
python3 scripts/compare_free_space_wrench_runtime_residuals.py --self-check
python3 scripts/compare_free_space_wrench_runtime_residuals.py   --output /tmp/free_space_wrench_model_comparison_recheck
```
