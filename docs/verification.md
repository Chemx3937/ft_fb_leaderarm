# Verification

## Selection

- 문서만 변경하면 diff를 검토하고 문서 검증을 실행한다.
- 소스, 설정, launch, 빌드 동작을 변경하면 빌드와 전체 테스트를 실행한다.
- 하드웨어 동작은 소프트웨어 검증을 먼저 수행하고, 사용자 승인 없이 실행하지 않는다.

## Documentation

```bash
git status --short
git diff --check
git diff
```

## Build and test

```bash
cd /home/vision/dualarm_ws
source /opt/ros/humble/setup.bash
source /home/vision/contact_pipeline_ws/install/setup.bash
colcon build --symlink-install --packages-select ft_fb_leaderarm
source /home/vision/dualarm_ws/install/setup.bash
export PYTHONPATH=/home/vision/venv_act/lib/python3.10/site-packages:$PYTHONPATH
colcon test --packages-select ft_fb_leaderarm --event-handlers console_direct+
colcon test-result --test-result-base build/ft_fb_leaderarm --verbose
```

현재 PC의 CTest는 system `pytest`를 사용하지만 PyTorch는 `venv_act`에 있으므로
전체 테스트 전에 위 `PYTHONPATH`가 필요하다. `contact_observer_msgs`는
`contact_pipeline_ws` 환경에서 제공된다.

## Observer runtime and FREE evidence

사용자 승인 아래 observer가 실행 중일 때만 다음 passive 평가를 실행한다. 같은
report에서 `FS-05` runtime과 `FS-06` FREE residual을 판정한다.

```bash
ros2 run ft_fb_leaderarm ft_observer_runtime_evaluate \
  --duration-s 10 --output /absolute/path/observer_runtime.json
```

완료 보고에는 실행한 명령, 통과·실패 결과, 실행하지 못한 하드웨어 검증을 기록한다.
