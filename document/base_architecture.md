# Free-space wrench 학습 architecture

## 공통 문제 정의

모든 후보는 같은 입력 source와 target을 사용한다.

```text
base feature: q[6], dq[6], causal qdd[6]
target      : physical FT [Fx,Fy,Fz,Mx,My,Mz] in aft_sensor2 frame
sample rate : 262.5 Hz
```

q는 주기성을 표현하기 위해 모델 입력 직전에 `sin(q), cos(q)`로 변환한다.
qdd는 미래 sample 없이 이전 dq와 현재 dq의 차이로 계산한다.

## 1. `static_linear`

### 입력과 구조

```text
현재 sin(q), cos(q): 12차원
linear layer       : 12 → 6
```

### 목적

자세에 따른 payload 중력 성분만으로 free-space wrench를 어느 정도 설명할 수
있는지 확인하는 가장 단순한 baseline이다.

### 장단점

- 학습과 inference가 가장 빠르다.
- 데이터가 적어도 비교적 안정적이다.
- 속도, 가속도, sensor dynamics를 표현하지 못한다.
- 이 모델이 1 N을 통과하면 더 복잡한 모델을 선택할 이유가 작다.

## 2. `dynamic_mlp`

### 입력과 구조

```text
현재 sin(q), cos(q), dq, qdd: 24차원
MLP hidden                    : 128 → 128
output                        : 6
```

### 목적

현재 순간의 자세·속도·가속도로 중력, 관성, 속도 의존 비선형 효과를
예측한다.

### 장단점

- history 없이 동적 효과를 표현한다.
- recurrent 모델보다 inference와 debugging이 단순하다.
- 같은 현재 상태라도 과거 운동 방향에 따라 sensor 값이 달라지는 현상은
  구분하기 어렵다.

## 3. `history_mlp`

### 입력과 구조

```text
최근 16 sample × 24 feature
flatten: 384차원
MLP hidden: 128 → 128
output: 6
```

16 sample은 약 61 ms의 causal window다. 미래 데이터는 사용하지 않는다.

### 목적

센서 지연, 짧은 진동, acceleration 계산 noise처럼 최근 이력이 필요한
현상을 recurrent state 없이 표현한다.

### 장단점

- MLP라 학습과 export가 단순하다.
- 시간 순서를 별도 구조로 학습하지 않고 고정 위치 feature로 취급한다.
- 입력 차원이 커져 데이터 요구량이 증가한다.

## 4. `history_lstm`

### 입력과 구조

```text
sequence: 16 × 24
one-layer LSTM hidden: 128
last hidden → linear 6
```

### 목적

최근 motion history에서 필요한 정보를 gate로 유지·삭제하면서 hysteresis,
sensor settling, 비선형 시간 의존성을 학습한다.

### 장단점

- 복잡한 시간 의존성을 표현할 가능성이 가장 크다.
- parameter와 연산량이 증가한다.
- 작은 dataset에서는 과적합할 수 있다.
- 성능이 좋아도 p99 3.048 ms 또는 hard max 3.810 ms runtime deadline을 넘으면 사용할 수 없다.

## 5. `history_gru`

### 입력과 구조

```text
sequence: 16 × 24
one-layer GRU hidden: 128
last hidden → linear 6
```

### 목적

LSTM과 같은 causal sequence를 더 단순한 recurrent gate로 처리한다.

### 장단점

- 일반적으로 LSTM보다 parameter와 연산이 적다.
- 같은 정확도라면 262.5 Hz runtime에서 유리할 수 있다.
- 실제 FT 데이터에서는 LSTM보다 항상 좋거나 나쁘다고 미리 결정할 수 없다.

## 공통 학습 방식

- train/validation/test는 episode가 아니라 `zero_set_id` group으로 분리한다.
- loss는 전체 6축 MSE와 상위 5% force tail 오차를 함께 사용한다.
- checkpoint 선택은 validation RMSE로 수행한다.
- 최종 후보 선택은 robust validation gate를 통과한 후보 중 force RMSE 최저다.
- 선택 후 held-out test를 한 번 평가한다.
- 모델과 normalization을 하나의 TorchScript `model.ts`로 저장한다.

## 승인 기준

```text
validation/test aggregate p99 ||Fraw-Fpred||2 <= 1.0 N
validation/test every zero-set group p95 <= 1.0 N
validation/test hard max <= 2.0 N
inference p99 <= 3.048 ms
inference hard max <= 3.810 ms
```

현재 승인은 force 3축 기준이다. 모델은 moment도 예측하지만 moment에 대한
승인 한계는 아직 정의되지 않았다. moment feedback을 사용할 경우 별도 Nm
기준을 먼저 정해야 한다.

## 사용하지 않는 입력

다음 값은 contact leakage 위험 때문에 runtime 입력에서 금지한다.

- physical/JTS measured wrench
- measured/raw joint torque
- task error
- 미래 q, dq 또는 centered derivative

raw wrench는 정답으로만 사용한다. 접촉이 포함된 episode를 학습 target으로
넣으면 모델이 접촉까지 free-space로 예측하므로 해당 episode는 폐기한다.
