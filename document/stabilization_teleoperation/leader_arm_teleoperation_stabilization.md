# Leader Arm Teleoperation Stabilization & Clean Demonstration Data Pipeline

## 1. 목적

현재 Leader Arm 기반 teleoperation 시스템에서는 다음 두 종류의 비의도성 움직임이 발생한다.

1. **Force feedback에 의한 고주파 진동**
   - F/T sensor 기반 force feedback torque가 Leader Arm에 인가될 때 Leader가 진동함.
   - Leader의 진동이 그대로 Follower Robot command에 전달되어 Robot trajectory에도 진동이 포함됨.
   - 이 trajectory를 imitation learning demonstration으로 저장하면 policy가 불필요한 진동을 학습할 수 있음.

2. **Gravity compensation에 의한 저주파 artifact**
   - Leader Arm에 적용된 gravity compensation torque가 실제 중력을 완전히 상쇄하지 못하거나 과보상함.
   - 사용자가 Leader를 움직일 때 특정 자세에서 갑자기 밀리거나, 끊기거나, drift가 발생할 수 있음.
   - 결과적으로 사용자가 의도한 smooth trajectory와 실제 Leader trajectory가 달라짐.

최종 목표는 다음과 같다.

> Leader의 raw trajectory를 그대로 Follower에 전달하지 않고,
> **사용자의 의도된 smooth trajectory (`leader_intent`)를 생성하여**
> 1) Follower command로 사용하고
> 2) imitation learning의 expert action label로 저장한다.

---

# 2. 전체 시스템 구조

```text
                        ┌─────────────────────────────┐
                        │      Leader Controller       │
                        │                             │
Human ────────> Leader  │  Gravity Compensation      │
                        │  + Force Feedback           │
                        │  + Virtual Damping          │
                        └──────────────┬──────────────┘
                                       │
                                       ▼
                             leader_raw_pose
                                       │
                                       ▼
                    ┌──────────────────────────────┐
                    │ Human Intent Trajectory       │
                    │ Generator / Stabilizer        │
                    │                               │
                    │ - vibration suppression       │
                    │ - smoothing                   │
                    │ - velocity limit              │
                    │ - acceleration limit          │
                    │ - optional jerk limit         │
                    └──────────────┬───────────────┘
                                   │
                                   ▼
                            leader_intent
                                   │
                         ┌─────────┴──────────┐
                         ▼                    ▼
                Follower command       Training action
                         │
                         ▼
                    Follower Robot
                         │
                         ▼
                     F/T Sensor
                         │
                         ▼
                  Contact Observer
                         │
                         ▼
               Contact Wrench Estimate
                         │
                         ▼
                Force Feedback Filter
                         │
                         ▼
                    Leader Arm
```

---

# 3. Leader Arm Torque 구성

Leader torque는 기본적으로 다음 구조를 사용한다.

```text
tau_leader
    =
gravity_compensation
+ force_feedback
- virtual_damping
```

수식으로는 다음과 같이 생각한다.

```text
tau_leader =
alpha * tau_gravity(q)
+ J(q)^T * Gf * F_contact_filtered
- B * dq
```

각 항의 의미:

- `tau_gravity(q)`
  - robot model 기반 gravity compensation torque

- `alpha`
  - gravity compensation scale factor
  - 기본값 1.0이 반드시 최적일 필요는 없음.
  - 예: 1.0 / 0.9 / 0.8 / 0.7 등을 실험하여 가장 안정적인 값 선택

- `F_contact_filtered`
  - F/T sensor raw wrench를 그대로 사용하지 않고 filtering 및 self-motion compensation을 수행한 contact wrench

- `Gf`
  - force feedback gain

- `J(q)^T`
  - Cartesian wrench를 Leader joint torque로 변환

- `B * dq`
  - virtual damping
  - Leader가 스스로 튀거나 진동하는 것을 억제

---

# 4. Gravity Compensation 처리

## 4.1 목적

Gravity compensation은 사용자가 Leader Arm의 무게를 거의 느끼지 않도록 하는 것이 목적이다.

하지만 과보상되면 Leader가 스스로 움직이고,
부족하면 사용자가 계속 중력 방향의 무게를 견뎌야 한다.

따라서 다음을 실험한다.

```text
alpha = 1.0
alpha = 0.9
alpha = 0.8
alpha = 0.7
```

Leader를 여러 자세에 정지시킨 뒤 손을 놓고 다음을 확인한다.

- 위로 움직임 → gravity compensation 과다
- 아래로 떨어짐 → gravity compensation 부족
- 자세마다 방향이 다름 → mass / CoM / payload model 오차 가능성

---

## 4.2 Virtual Damping 추가

Gravity compensation만 적용하면 residual torque로 인해 Leader가 drift하거나 튈 수 있다.

따라서 다음 damping torque를 추가한다.

```text
tau_damping = -B * dq
```

최종적으로:

```text
tau_leader =
alpha * tau_gravity(q)
+ tau_force_feedback
- B * dq
```

주의:

- `B`가 너무 작으면 damping 효과가 부족
- `B`가 너무 크면 사용자가 Leader를 움직일 때 지나치게 뻑뻑해짐

따라서 최소한의 damping으로 vibration/drift를 억제하는 값을 찾는다.

---

# 5. Force Feedback 처리

## 5.1 Raw F/T 값을 바로 사용하지 않는다

다음 pipeline을 사용한다.

```text
FT raw wrench
        ↓
Bias removal
        ↓
Self-motion wrench compensation
        ↓
Contact wrench estimation
        ↓
Low-pass / optional notch filter
        ↓
Force feedback gain
        ↓
Saturation
        ↓
Torque slew-rate limit
        ↓
J^T
        ↓
Leader torque
```

---

## 5.2 Self-motion Wrench Compensation

Follower Robot이 free-space에서 움직이더라도 F/T sensor에서는 robot motion에 의한 wrench가 측정될 수 있다.

따라서:

```text
F_contact =
F_measured - F_self_motion_estimated
```

`F_self_motion_estimated`는 기존 Contact Observer 또는 learned wrench estimator를 통해 계산한다.

---

## 5.3 Contact State Chattering 방지

Contact state가 다음과 같이 빠르게 ON/OFF 반복되면:

```text
0 → 1 → 0 → 1 → 0 → 1
```

force feedback torque도 반복적으로 켜졌다 꺼져 진동을 만들 수 있다.

따라서 다음 중 하나 이상을 적용한다.

### Hysteresis

```text
contact_on_threshold  > contact_off_threshold
```

예:

```text
ON  : confidence > 0.7
OFF : confidence < 0.3
```

### Consecutive sample 조건

예:

```text
contact condition이 5 sample 연속 유지될 때 ON
non-contact condition이 5 sample 연속 유지될 때 OFF
```

### Gain ramp

Force feedback gain을 순간적으로 0 → 1로 바꾸지 않는다.

```text
g_next =
g_current + clamp(
    g_target - g_current,
    -gain_rate * dt,
     gain_rate * dt
)
```

그리고

```text
tau_feedback =
g * J^T * F_contact_filtered
```

형태로 적용한다.

---

# 6. Force Feedback Vibration 억제

## 6.1 먼저 진동 주파수를 측정한다

다음 세 조건에서 Leader / Follower trajectory를 logging한다.

```text
Case A: Force Feedback OFF
Case B: Force Feedback ON + Free Space
Case C: Force Feedback ON + Contact
```

각 축에 대해 PSD(Power Spectral Density)를 계산한다.

확인할 것:

- FF ON에서만 특정 frequency peak가 생기는지
- Contact에서만 peak가 커지는지
- 특정 axis만 진동하는지

---

## 6.2 Filtering 전략

### 특정 좁은 진동 주파수가 존재

```text
Notch Filter
```

사용.

### 넓은 대역의 jitter

```text
Low-pass filter
```

또는

```text
One Euro Filter
```

사용 가능.

주의:

Filtering 목적은 사용자의 실제 의도 동작까지 없애는 것이 아니라
**feedback controller가 만든 high-frequency artifact만 제거하는 것**이다.

---

# 7. Human Intent Trajectory Generator

이 모듈이 핵심이다.

Follower Robot이 `leader_raw`를 직접 따라가지 않도록 한다.

기존:

```text
leader_raw
    ↓
coordinate transform
    ↓
follower command
```

변경:

```text
leader_raw
    ↓
coordinate transform
    ↓
Human Intent Trajectory Generator
    ↓
leader_intent
    ↓
follower command
```

---

# 8. Intent Trajectory Generator 구현 방법

## 권장 방식: 2차 Reference Generator

각 Cartesian axis 또는 joint axis에 대해 다음 구조를 사용한다.

```text
xddot_intent =
wn^2 * (x_raw - x_intent)
- 2 * zeta * wn * xdot_intent
```

개념적으로:

```text
Leader Raw
     ↓
Virtual Spring-Damper
     ↓
Leader Intent
```

장점:

- 작은 고주파 vibration 억제
- gravity compensation에 의한 순간적인 movement spike 완화
- 사용자의 느리고 큰 intentional motion은 유지

---

## 8.1 Velocity Limit

```text
|xdot_intent| <= v_max
```

---

## 8.2 Acceleration Limit

```text
|xddot_intent| <= a_max
```

---

## 8.3 Optional Jerk Limit

필요하다면:

```text
|xjerk_intent| <= j_max
```

Gravity compensation artifact는 position noise보다
velocity / acceleration spike 형태로 보일 수 있으므로
acceleration / jerk limit가 중요하다.

---

# 9. Online Teleoperation Pipeline

최종적으로 Leader → Follower command는 다음 순서를 사용한다.

```text
Leader measured pose
        ↓
Coordinate transformation
        ↓
Optional notch filtering
        ↓
Reference / intent trajectory generator
        ↓
Velocity limiting
        ↓
Acceleration limiting
        ↓
Optional jerk limiting
        ↓
Follower command
```

Follower에는 `leader_raw`가 아니라
항상 `leader_intent`를 전달한다.

---

# 10. Demonstration Data 저장

다음 데이터를 모두 저장한다.

```text
timestamp
leader_raw_pose
leader_raw_velocity

leader_intent_pose
leader_intent_velocity

follower_command

robot_measured_state
robot_measured_velocity

ft_raw_wrench
ft_contact_wrench

contact_state
contact_confidence

rgb_image
depth_image (if available)
```

---

# 11. Imitation Learning에서 사용할 데이터

## Observation

예:

```text
observation_t = {
    image_t,
    robot_state_t,
    contact_wrench_t,
    contact_state_t
}
```

---

## Action Label

사용하면 안 되는 것:

```text
leader_raw
```

권장:

```text
leader_intent
```

또는 follower controller에 실제 전달한:

```text
follower_command
```

즉:

```text
expert_action = clean_follower_command
```

---

# 12. Diffusion Policy 사용 시

Action horizon이 H라면:

```text
action_t = [
    clean_action_(t+1),
    clean_action_(t+2),
    ...
    clean_action_(t+H)
]
```

Delta action을 사용하는 경우에는 가능하면
현재 실제 robot state를 기준으로 target을 정의한다.

```text
delta_action =
clean_target - current_robot_measured_state
```

즉:

```text
clean_target - previous_clean_target
```

보다 실제 robot state 기준이 바람직하다.

---

# 13. 기존 저장 데이터 후처리

이미 진동이 포함된 dataset이 있다면 우선순위는 다음과 같다.

```text
1. 저장된 leader raw command를 smoothing
2. leader raw pose를 follower frame으로 변환 후 smoothing
3. 위 두 개가 없으면 robot measured trajectory를 smoothing
```

가능하면 `robot_measured_state` 자체를 expert label로 smoothing하는 것보다
**leader command / intended command를 reconstruction하는 방식**을 우선한다.

왜냐하면 robot measured state에는 다음이 모두 섞여 있기 때문이다.

```text
leader vibration
robot tracking error
contact deformation
low-level controller dynamics
sensor noise
```

---

# 14. Offline Smoothing

Offline dataset processing에서는 zero-phase filtering을 사용할 수 있다.

예:

```python
from scipy.signal import butter, sosfiltfilt

sos = butter(
    N=4,
    Wn=cutoff_hz,
    btype="lowpass",
    fs=sampling_hz,
    output="sos"
)

clean_traj = sosfiltfilt(
    sos,
    raw_traj,
    axis=0
)
```

주의:

`filtfilt` 계열은 미래 sample을 사용하므로
실시간 제어에는 사용할 수 없다.

사용처:

```text
Offline dataset action relabeling : 가능
Online teleoperation               : 불가
Online inference preprocessing     : 불가
```

---

# 15. Contact-rich Task에서 주의할 점

단순 smoothing을 너무 강하게 적용하면 다음 중요한 event가 이동할 수 있다.

```text
contact start
grasp timing
insertion timing
tool touch timing
task waypoint
```

따라서 contact transition 주변에서는 filtering을 약하게 하거나
중요 waypoint를 constraint로 유지하는 것이 좋다.

예:

```text
Free-space:
    strong smoothing

Near contact transition:
    weak smoothing

During precision contact:
    preserve task-relevant motion
```

---

# 16. Friction Compensation

현재 단계에서는 **필수 구현 항목이 아니다.**

우선순위:

```text
1. Gravity compensation tuning
2. Virtual damping
3. Force feedback tuning
4. Force feedback filtering
5. Human intent trajectory generator
6. Velocity / acceleration / jerk limiting
7. 필요할 때만 friction compensation
```

다음 증상이 명확할 때만 friction compensation을 검토한다.

- 아주 천천히 움직일 때 일정 힘 이상 줘야 Leader가 움직임
- 움직이기 시작하면 갑자기 `툭` 움직임
- 방향 전환 시 dead-zone 느낌
- low-speed stick-slip 현상

주의:

잘못된 friction compensation은 오히려

- over-compensation
- self-motion
- vibration amplification

을 만들 수 있다.

따라서 지금은 friction compensation 없이 먼저 안정화한다.

---

# 17. 구현 권장 순서

## Phase 1 — Logging

먼저 시스템을 수정하지 않고 다음을 모두 logging한다.

```text
timestamp
leader q
leader dq
leader command torque
gravity compensation torque
force feedback torque
follower q
follower dq
follower command
FT wrench
contact state
```

---

## Phase 2 — Gravity Compensation 안정화

다음 조건 비교:

```text
alpha = 1.0
alpha = 0.9
alpha = 0.8
alpha = 0.7
```

각 조건에서:

```text
leader drift
leader velocity spike
trajectory smoothness
user effort
```

비교.

---

## Phase 3 — Virtual Damping

```text
tau_damping = -B * dq
```

추가 후 B 값을 증가시키며 테스트.

목표:

```text
최소한의 사용자 저항
+
충분한 vibration / drift suppression
```

---

## Phase 4 — Force Feedback 안정화

비교:

```text
Gf = 1.0
Gf = 0.7
Gf = 0.5
Gf = 0.3
```

추가:

```text
low-pass
notch
saturation
torque slew-rate limit
contact hysteresis
gain ramp
```

---

## Phase 5 — Intent Trajectory Generator

Leader raw trajectory를 follower에 직접 전달하지 않는다.

```text
leader_raw
   ↓
intent generator
   ↓
leader_intent
   ↓
follower command
```

---

## Phase 6 — Demonstration Data 저장

Policy action target:

```text
leader_intent
```

또는

```text
actual follower command
```

사용.

---

# 18. 정량 평가 지표

각 실험 조건에 대해 다음 값을 계산한다.

## Trajectory Smoothness

### Velocity variance

```text
Var(dq)
```

### Acceleration RMS

```text
RMS(ddq)
```

### Jerk RMS

```text
RMS(dddq)
```

### High-frequency trajectory energy

PSD에서 vibration frequency band의 energy 계산.

---

## Tracking

```text
RMSE(
    follower_measured,
    follower_command
)
```

---

## Force Feedback

```text
contact force peak
contact force RMS
force feedback torque RMS
```

---

## Task Performance

```text
task success rate
completion time
maximum contact force
trajectory length
```

---

# 19. 중요한 구현 원칙

## 원칙 1

```text
Leader raw trajectory != Expert action
```

---

## 원칙 2

Follower Robot은 항상 clean reference를 따라야 한다.

```text
Leader Raw
    ↓
Intent Generator
    ↓
Follower Command
```

---

## 원칙 3

Force feedback loop와 motion command loop를 논리적으로 분리한다.

```text
Motion:
Leader → Intent Filter → Follower

Haptic:
Follower F/T → Contact Wrench → Leader
```

---

## 원칙 4

Filtering으로 문제를 숨기기 전에
gravity compensation과 force feedback controller 자체를 먼저 안정화한다.

---

## 원칙 5

마찰보상은 현재 필수 항목이 아니다.

중력보상 + damping + force feedback 안정화 이후에도
low-speed stick-slip이 명확하게 남는 경우에만 추가한다.

---

# 20. 최종 목표 Architecture

```text
┌───────────────────────────────────────────────────────┐
│                       LEADER                          │
│                                                       │
│  Human                                                │
│    ↓                                                  │
│  Leader Arm                                           │
│    ↑                                                  │
│    ├── Gravity Compensation × alpha                  │
│    ├── Filtered Force Feedback                       │
│    └── Virtual Damping                               │
│                                                       │
│              ↓ leader_raw                            │
└──────────────┼────────────────────────────────────────┘
               │
               ▼
┌───────────────────────────────────────────────────────┐
│            HUMAN INTENT TRAJECTORY GENERATOR          │
│                                                       │
│  - vibration filtering                               │
│  - reference generator                               │
│  - velocity limit                                    │
│  - acceleration limit                                │
│  - optional jerk limit                               │
│                                                       │
│              ↓ leader_intent                         │
└──────────────┼────────────────────────────────────────┘
               │
               ├──────────────→ Save as Expert Action
               │
               ▼
┌───────────────────────────────────────────────────────┐
│                      FOLLOWER                         │
│                                                       │
│  Robot Controller                                    │
│       ↓                                               │
│  Follower Robot                                      │
│       ↓                                               │
│  F/T Sensor                                          │
│       ↓                                               │
│  Contact Observer                                    │
│       ↓                                               │
│  Contact Wrench Filtering                            │
│       ↓                                               │
└───────┼───────────────────────────────────────────────┘
        │
        └──────────────→ Leader Force Feedback
```

---

# 21. Codex 구현 시 우선 요청사항

Codex는 기존 코드 구조를 먼저 분석한 뒤,
가능하면 기존 teleoperation/control 구조를 크게 깨지 않는 방향으로 구현한다.

우선 다음 모듈을 분리해서 구현하는 것을 권장한다.

```text
GravityCompensationController
ForceFeedbackController
ContactFeedbackFilter
IntentTrajectoryGenerator
MotionLimiter
TeleoperationLogger
```

각 모듈은 parameter를 config 파일 또는 ROS parameter로 조절할 수 있게 만든다.

예:

```yaml
leader_control:
  gravity_scale: 0.9

  virtual_damping:
    enabled: true
    coefficient: [...]

force_feedback:
  enabled: true
  gain: 0.5
  lowpass_cutoff_hz: ...
  notch_enabled: false
  notch_frequency_hz: ...
  max_force: ...
  max_torque_rate: ...

contact:
  on_threshold: ...
  off_threshold: ...
  debounce_samples: ...

intent_generator:
  enabled: true
  natural_frequency: ...
  damping_ratio: 1.0
  max_velocity: [...]
  max_acceleration: [...]
  max_jerk: [...]
```

모든 parameter의 초기값은 하드코딩하지 말고
기존 robot control frequency와 robot specification을 확인한 뒤 설정한다.

---

# 22. 구현 시 안전 관련 주의

Leader와 Follower가 실제 robot hardware이므로 다음을 반드시 유지한다.

```text
joint limit
velocity limit
acceleration limit
torque limit
force feedback saturation
watchdog
emergency stop behavior
```

새로운 filtering/controller를 추가하면서
기존 safety constraint를 bypass하지 않는다.

특히 gravity compensation, damping, force feedback torque가 합산된 최종 torque가
robot torque limit을 초과하지 않도록 saturation을 적용한다.

---

# 23. 최종 핵심

이 작업의 목적은 단순히 trajectory를 예쁘게 smoothing하는 것이 아니다.

```text
Leader의 실제 움직임
=
사용자 의도
+ gravity compensation artifact
+ force feedback vibration
+ 기타 controller artifact
```

이므로,

```text
Leader Raw
      ↓
User Intent Estimation / Stabilization
      ↓
Clean Demonstration
```

구조를 만드는 것이 핵심이다.

최종적으로 imitation learning policy는
controller artifact가 포함된 trajectory가 아니라
**사용자가 실제로 의도한 clean follower command를 학습해야 한다.**
