# 현재 Free-space wrench 모델 pseudocode

## 동작 순서 요약

1. 모델·metadata·URDF의 SHA와 runtime 계약을 검증한다.
2. Robot state와 AFT sample이 valid, fresh, synchronized인지 확인한다.
3. 고정 zero pose에서 1초간 자세·정지·force 안정성을 확인한다.
4. `[q,dq,causal-qdd]`를 최근 32 sample까지 쌓는다.
5. 물리 payload gravity와 54D ridge residual을 각각 계산해 더한다.
6. 측정 AFT에서 예측 free-space wrench를 빼 contact residual을 만든다.
7. force norm에 Schmitt threshold와 hold time을 적용한다.
8. base-frame observation과 sensor-frame 분석 topic을 발행한다.

## 1. Bundle 로딩

```text
function load_bundle(model_path):
    metadata_path = sibling(model_path, "metadata.json")
    require file_exists(model_path, metadata_path)

    metadata = parse_json(metadata_path)
    model_hash = sha256(model_path)
    metadata_hash = sha256(metadata_path)

    require metadata.schema_version == 1
    require metadata.sample_hz == 262.5
    require metadata.base_feature_dim == 18
    require metadata.model_sha256 == model_hash

    acceptance_source = one_of:
        metadata.approved == true
            and metadata.approval_contract == "robust_force_v2_262p5hz"
        exact(model_hash, metadata_hash)
            == operator_selected_20260901 hashes
    otherwise reject

    require metadata.ablation == "physical_residual_short_multiscale_ridge"
    require metadata.feature_mode == "short_multiscale"
    require metadata.history == 32
    require metadata.architecture == "ridge"

    require sha256(configured_urdf) == metadata.gravity_model.urdf_sha256
    ridge_model = torchscript_load(model_path)
    gravity_model = make_pinocchio_payload_model(metadata.gravity_model)

    return BundlePredictor(ridge_model, gravity_model, metadata,
                           acceptance_source)
```

## 2. Observer 시작

```text
function start_observer(parameters):
    require parameters.zero_set_confirmed == true
    require parameters.zero_set_id is not empty
    require parameters.sample_hz == 262.5

    predictor = load_bundle(parameters.model_path)

    require parameter frame/payload/controller/zero_pose
            exactly match predictor.metadata

    benchmark predictor for 200 calls after 20 warm-up calls
    require p99_inference <= 0.8 * 3.80952 ms
    require max_inference <= 3.80952 ms

    zero_verified = false
    history = deque(max_length=32)
    detector = Schmitt(on=3.0 N, off=1.2 N,
                       on_hold=20 ms, off_hold=20 ms)
    start timer at 262.5 Hz
```

## 3. 입력 callback

```text
on ObserverInput message:
    save q[6], dq[6], current_pose[6], source_sequence, timestamp
    valid = message.valid
            and frame == "right_base_link"
            and every value is finite

on AFT WrenchStamped message:
    save W_sensor[6], timestamp
    valid = frame == "aft_sensor2"
            and every value is finite
```

## 4. 매 주기 입력 검증

```text
function validate_pair(robot, ft, now):
    reject if either stream is missing or invalid
    reject if either source age > 20 ms
    reject if either local receive age > 20 ms
    reject if timestamp is more than 2 ms in the future
    reject if abs(robot.timestamp - ft.timestamp) > 3 ms
    reject if robot.source_sequence was already processed
    otherwise accept
```

`duplicate_robot_source`를 제외한 실패에서는 feature/history/detector를 초기화한다.
모든 실패는 `valid=false` observation으로 발행한다.

## 5. Fixed-zero 확인과 feature 생성

```text
if not zero_verified:
    add current sample to 1-second zero window only when:
        max_abs(q - zero_pose) <= 1 degree
        max_abs(dq) <= 0.02 rad/s
    require at least 100 samples
    require norm(median(force_window)) <= 1 N
    require every force-axis std <= 0.40 N
    if not ready:
        publish_invalid(zero_reason)
        return

qdd = zeros(6)
if 0 < current_stamp - previous_stamp <= 50 ms:
    qdd = (dq - previous_dq) / dt

history.append([q, dq, qdd])
if history.length < 32:
    publish_invalid("history_warmup")
    return
```

## 6. Model prediction

```text
function predict(history[32,18]):
    q_now = history[-1].q
    dq_now = history[-1].dq

    x54 = concatenate(
        sin(q_now),
        cos(q_now),
        dq_now,
        mean(history[-8:].dq),
        mean(history[-16:].dq),
        mean(history[-32:].dq),
        mean(history[-8:].qdd),
        mean(history[-16:].qdd),
        mean(history[-32:].qdd),
    )

    W_ridge = torchscript_normalized_linear(x54)

    gravity_now = gravity_vector_in_sensor_frame(q_now)
    gravity_zero = gravity_vector_in_sensor_frame(zero_pose)
    F_gravity = mass * (gravity_now - gravity_zero)
    M_gravity = cross(com_sensor, F_gravity)
    W_gravity = concatenate(F_gravity, M_gravity)

    return W_gravity + W_ridge
```

## 7. Contact residual과 detector

```text
W_free_hat = predict(history)
W_contact_sensor = W_sensor - W_free_hat
e_force = norm(W_contact_sensor.force)

if detector.state == FREE:
    if e_force >= 3.0 N continuously for 20 ms:
        detector.state = CONTACT
else:
    if e_force <= 1.2 N continuously for 20 ms:
        detector.state = FREE

if 1.2 N < e_force < 3.0 N:
    keep detector.state
```

## 8. Frame 변환과 발행

```text
W_free_base = transform_sensor_wrench_to_base(W_free_hat, current_pose,
                                               sensor_to_tip_transform)
W_contact_base = transform_sensor_wrench_to_base(W_contact_sensor, current_pose,
                                                  sensor_to_tip_transform)

publish ContactObservation:
    frame = right_base_link
    contact_wrench = W_contact_base
    free_space_wrench_prediction = W_free_base
    contact_state = detector.state
    contact_score = e_force
    valid = true
    model_ready = true
    source/prediction sequence and latency fields

publish sensor-frame W_free_hat and W_contact_sensor for diagnostics
```

## 9. Invalid/failure 처리

```text
on stale, sync failure, warm-up, zero failure, or inference exception:
    publish ContactObservation:
        contact_wrench = zeros(6)
        free_space_wrench_prediction = zeros(6)
        contact_state = FREE
        valid = false
        model_ready = true

    feedback consumer must apply exactly zero reflected torque
```

`contact_state=FREE` 값만 보고 정상 FREE로 해석하면 안 된다. 반드시 `valid`와
`model_ready`를 함께 확인한다.

## 10. 학습 pseudocode

```text
load train groups and protected validation groups
compute sensor-frame gravity delta from URDF for every q

select train samples in the lowest acceleration quartile
fit payload mass from F = mass * delta_gravity
fit sensor-frame CoM from M = CoM × F

for every sample with 32-sample history:
    W_gravity = physical_payload_prediction(q)
    residual_target = measured_wrench - W_gravity
    x54 = short_multiscale_features(history)

standardize x54 and residual_target using train statistics
fit ridge coefficients with L2 regularization = 1.0
export normalization + linear coefficients as TorchScript model.ts
write model, data, frame, payload, controller, URDF and benchmark contracts
```

## 관련 문서

- 설명과 구조도: [free_space_wrench_model_architecture.md](free_space_wrench_model_architecture.md)
- 현재 운용 순서: [flow.md](flow.md)
- 실행 명령: [command.md](command.md)
