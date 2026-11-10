# 세 모델 점증 가압 20회와 실시간 판정 검증

현재 계획은 **기존 무접촉 20회 + 새 점증 가압·해제 20회**다. 약한·일반 접촉의
추가 수집은 진행하지 않는다. 기존 예비·약한·혼합 하중·노이즈 기록과 당시 세션
스냅샷은 보존한다. 새 물리 기록 하나를 세 모델에 공통으로 사용한다.

현재 세션은 `data/three_model_comparison/session_20261111_05/session.json`이고
평가 모드는 `ramp_detection_live_v1`이다. 같은 zero-set·센서·박스 파지를 유지
중이라는 운용자 확인에 따라 이 세션의 정지 노이즈 PASS 기준선을 재사용한다.
zero-set이나 센서·파지가 달라졌다면 다른 zero-set 세션과 정지 노이즈가 필요하다.
GUI는 모터·teleoperation·feedback·FT zero-set 명령을 실행하지 않는다.

2026-11-11에 확정한 FREE 20회와 대체 수집을 포함한 RAMP 20회의 결과는
[정량 평가 보고서](../data/three_model_comparison/evaluation_20261111_free20_ramp20/report.html)에
있다. 회차별 CSV, 고정 폴더 목록, SHA 검증과 재현용 평가 스크립트를 함께 저장했다.
세 모델 모두 가압 중 첫 접촉은 20/20회 검출했고 관측된 판정 규칙은 일치했다.
입력 시각 불일치로 CONTACT가 초기화되는 현상은 별도의 runtime 한계로 보고했다.
이 분석은 모델·detector·제어 설정이나 정식 승인 상태를 변경하지 않는다.

## 평가 목적과 센서 역할

| 목적 | 사용 기록 | 결과 |
|---|---|---|
| 몇 N에서 검출하는가 | 실제 실시간 observer의 첫 새 CONTACT 완료 시각 + 외부 F/T 원본 | 검출 하중 min/median/max/std, 검출·미검출·사전 활성·동기 불량 횟수 |
| 설정대로 판정하는가 | 실시간 손목 실측·예측·잔차·상태·시각 | ON/OFF·20 ms hold·히스테리시스·잔차 계산 일치 여부 |
| 모델의 무접촉 예측 오차·오검출 | 이전에 완료한 무접촉 20회 | 기존 RMSE/p99/max와 false CONTACT 결과 유지 |

- CONTACT/FREE는 **손목 실제 force − 각 모델의 free-space 예측 force의 norm**으로
  판정한다. ON/OFF 3.0/1.2 N, CONTACT/FREE hold 20/20 ms를 그대로 사용한다.
- 외부 `/aft_sensor1/wrench`는 하중계다. 외부 센서의 1.6 N 등의 별도 CONTACT
  문턱으로 손목 검출을 인정하거나 거르지 않는다. 외부 force와 손목 residual이
  같은 값이라고 가정하지 않는다.
- 외부 힘은 현재 zero-set의 무부하 중앙값 vector를 빼고 norm을 계산한다.
  축별 값은 손목 좌표계와 직접 겹치지 않는다. 손목에는 hardware zero 외의
  software bias를 추가하지 않는다.
- 세 모델의 학습·controller·payload·SHA와 native 전처리/history는 유지한다.
  이 GUI 내부의 진단용 observer 세 개를 평가하며 canonical ContactObservation
  publisher나 feedback 운용 승인을 새로 만드는 기능이 아니다.

## 실행 명령

리더암·핸드·두 F/T가 실행 중이면 유지하고 **기록·분석을 끝낸 뒤 기존 GUI만
종료**한다. 리더암은 feedback OFF 및 FAST 상태여야 한다. 리더암 startup script를
재실행하면 자동 정렬 동작이 발생하므로 단순 GUI 갱신을 위해 재실행하지 않는다.

실험 직전에 기존 PC alias로 시계 상태를 확인한다. 필요 시 sudo 인증은 사용자
터미널에서 처리한다.

```bash
dualarm_time_status
```

GO를 확인하고, 새 터미널에서 GUI를 실행한다.

```bash
source /opt/ros/humble/setup.bash
source /home/vision/contact_pipeline_ws/install/setup.bash
source /home/vision/dualarm_ws/install/setup.bash
ros2 launch ft_fb_leaderarm three_model_comparison_gui.launch.py session:=/home/vision/dualarm_ws/src/ft_fb_leaderarm/data/three_model_comparison/session_20261111_05/session.json
```

기본 선택은 `점증 가압·해제 (공통 20회)`다. 필요할 때만 `10초 정지 센서 노이즈`를
선택한다. 실제 무접촉, 유효한 입력, 저장된 노이즈 PASS, Teleop FAST를 확인한다.
초기 모델 CONTACT를 이유로 수집을 막거나 자료를 제외하지 않는다.

## 한 회차의 진행 순서

1. 매회 같은 무접촉 시작 위치·자세에서 박스를 외부 센서와 주변에서 완전히
   떨어뜨리고 `기록 시작`을 누른다.
2. 준비 2초 → **실제 무접촉 3초**를 유지한다. 초기 사전 활성도 함께 기록한다.
3. 접근 안내가 나오면 같은 접촉 위치·방향에서 비슷한 속도로 천천히 가압한다.
   어느 모델이 CONTACT가 되어도 계속 계획한 가압을 수행한다.
4. **외부 하중 30 N 도달 시 가압 증가를 멈추고**, 서서히 힘을 줄여 완전히
   떨어진다. 30 N에서 로봇이 자동 정지하거나 힘을 제한하지 않는다.
5. 외부 하중이 저장된 무부하 노이즈 범위로 복귀하면 자동으로 마지막 3초를
   안내한다. 실제로 완전히 떨어져 무접촉을 유지한다.
6. 완료 안내 후 `기록 종료`를 누르고 원본·실시간 로그·분석 저장 완료를 기다린다.
7. 1~6을 20회 반복한다. 모델마다 따로 20회를 하지 않는다.

중간 버튼은 없다. 외부의 무부하 복귀 범위는 현재 정지 기록의 centered norm
최댓값 약 1.55 N이며, 0.2초 복귀 확인 후 3초 안내를 시작한다. 이는 **해제 후
카운트다운을 돕는 조건**이고 접촉 정답 라벨이나 검출 하중 필터가 아니다.
범위를 벗어나거나 입력이 없으면 마지막 3초를 다시 센다. 작은 힘만으로 실제
분리 여부를 증명할 수 없으므로 물리적인 완전 해제는 운용자가 확인한다.
모델이 CONTACT를 유지해도 외부 하중의 무부하 복귀 안내를 차단하지 않는다.

기록은 최대 120초다. 시간 초과·중도 종료·로그 누락은 원본을 보존한다. 실제
절차와 로그가 완결되지 않은 기록은 계획 횟수에서 빼며, 모델 미검출·사전 활성·
판정 기준 위반을 이유로 정상 수집된 시행을 제외하지 않는다. 파일명 timestamp로
각 물리 기록을 구분하고 정상 수집 20회에서 추가 시작을 차단한다.

## GUI에서 확인할 것

- Fx/Fy/Fz 그래프: 흰색 손목 실측과 파랑·보라·주황의 세 모델 예측.
- 표: 손목 실측·각 모델 예측의 현재값, residual norm과 실제 CONTACT/FREE.
- 큰 숫자: 외부 하중, 30 N까지 남은 하중·상한 도달/해제 안내.
- 외부 센서에 별도 CONTACT/FREE를 표시하지 않는다. 판정은 손목 observer 표에 있다.
- 단계·남은 시간은 자동 안내하며 실제 접근·가압·해제는 리더암으로 직접 수행한다.

## 저장과 계산

각 `trials/<timestamp>_ramp/`에 다음 파일을 보존한다.

| 파일 | 내용 |
|---|---|
| `raw/` | 로봇 observer_input, 손목 F/T, 외부 F/T 원본 rosbag |
| `live_decisions.jsonl` | 모든 실제 계산 callback의 원본 입력·예측·잔차·준비 상태·판정·시각·단계 |
| `live_trace_info.json` | 저장 수·누락·쓰기 오류·종료 확인·SHA-256 |
| `session.json`, `noise_baseline.json`, `trial.json` | 고정 모델/config/zero/baseline과 수집 범위·절차 |
| `comparison.npz`, `comparison_report.json` | 기록된 실시간 결과와 검출 하중·판정 기준 검사 |

로그는 GUI의 5 Hz 화면 갱신이 아니라 목표 262.5 Hz의 **실제 계산 callback마다**
저장한다. bounded queue와 별도 writer를 사용하며 누락·쓰기 오류는 실패로 표시하고
원본을 보존한다. 종료·분석이 끝나기 전에는 창을 닫지 않는다.

판정 기준 검사는 detector 구현을 다시 호출하지 않는 별도 계산으로 수행한다.
손목 실측−예측으로 잔차도 다시 확인한다. 시작 checkpoint의 CONTACT/pending hold를
기록하여 수집 직전부터 이어지던 상태를 임의로 초기화하지 않는다.
유효 입력 구간에서 ON/OFF·20 ms hold·중간 범위의 상태 유지를 확인하며 너무 이른
전환·늦은 전환을 보고한다. 20 ms 초과 계산 간격이나 증거 부족은 INCOMPLETE,
불일치는 VIOLATION, 관측된 유효 tick의 일치는 PASS다. 실제 간격과 계산 시간도
저장하므로 목표 주파수만으로 실시간 성능을 통과했다고 보지 않는다.

검출 하중은 운용자가 가압을 시작하도록 안내받은 구간에서 새 FREE→CONTACT가
나온 시각을 사용한다. 외부 하중을 별도 이진 판정하지 않는다. 계산 완료 시각
이전의 가장 최근 외부 센서 header sample을 최대 3 ms 이내에서 대응시키고,
해당 원본 force를 사용한다. source가 없으면 0 N 대신 동기 불량으로 보고한다.
수집 전부터 CONTACT이거나 30 N 도달 후에야 검출된 경우는 따로 보고한다.

상위 `data/three_model_comparison/`의 `live_ramp_summary.json`과
`live_ramp_onsets.csv`에는 새 프로토콜만 모은다. 과거 offline-replay-only 결과를
실시간 판정 기록으로 대체하거나 이번 20회에 섞지 않는다.

## 해석의 범위와 검증

검출 하중에는 고정 20 ms hold, 계산 시간, 가압 속도, 센서 노이즈, 모델 오차가
영향을 준다. 현재 장착·파지·zero-set·접촉 방향에서의 결과이며 보편적인 최소
검출력이나 최초 물리 접촉 시각의 지연을 의미하지 않는다. 정지 노이즈와 215 g
하중 확인은 전체 구간 절대 힘 교정을 대체하지 않는다.

코드와 테스트는 `comparison.py`, `comparison_gui.py`, `comparison_trace.py`,
`test/test_comparison.py`, `test/test_comparison_trace.py`에서 확인한다.
빌드·전체 검증 절차는 [검증 문서](../docs/verification.md)를 따른다.
실시간 로그 없이 과거 원본만 재생한 분석으로 실제 운용 판정 준수를 주장하지 않는다.
정확도·feedback 승격에 관한 기존 [합격 계약](../docs/acceptance-contract.md)은 유지한다.
