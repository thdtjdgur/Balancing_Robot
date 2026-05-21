# ESP32 밸런싱로봇

`ESP32-S3` 기반 2륜 밸런싱로봇의 제어 코드와 검증 자료를 정리한 저장소이다.

기지국에서 전달한 GPS 웨이포인트를 순서대로 따라가며 사람을 수색하는 구조를 목표로 한다. 로봇은 현재 위치와 목표점을 로컬 `x/y` 좌표로 변환한다. `MPPI` 제어기는 이 좌표를 바탕으로 목표 속도와 목표 각속도를 계산한다. 이 명령은 모터에 바로 전달되지 않는다. 속도, 자세, 요 제어용 `PID`를 거친 뒤 왼쪽과 오른쪽 바퀴의 `Vq` 명령으로 나뉜다. 마지막으로 엔코더 각도를 이용한 역 DQ 변환을 통해 BLDC 3상 전압으로 출력한다.

실물 로봇을 바로 튜닝하지 않는다. MATLAB에서 MPPI 명령 생성 로직을 먼저 확인한다. 이후 `SILS`와 `HILS`로 PID 제어 흐름과 ESP32 펌웨어 통신 구조를 검증한다. 마지막 단계에서 실물 로봇으로 외란 복귀, 목표 속도 추종, 블루투스 수동 조작을 확인한다.

---

## 전체 제어 흐름

```text
기지국 GPS 웨이포인트
-> GNSS 로컬 좌표 변환
-> 웨이포인트 관리자
-> MPPI 제어기: v_ref, w_ref 생성
-> 속도 PID: 목표 피치 생성
-> 피치 PID + 요 제어: 좌우 Vq 생성
-> 엔코더 각도 보정
-> 역 DQ 변환
-> BLDC 3상 전압 출력
```

MPPI는 목표점까지 이동하기 위한 상위 명령을 만든다. PID는 로봇이 넘어지지 않도록 속도 명령을 자세 제어로 바꾼다. DQ 변환은 계산된 전압 명령을 실제 BLDC 모터가 사용할 3상 전압으로 바꾸는 단계이다.

---

## 구현 범위

- ESP32-S3 기반 밸런싱로봇 제어 루프를 구현한다.
- 기지국 GPS 웨이포인트를 순서대로 따라가는 GNSS 미션 레이어를 구성한다.
- MPPI 제어기로 목표 속도와 목표 각속도를 생성한다.
- MATLAB MPPI 시뮬레이션으로 펌웨어 적용 전 명령 생성 로직을 확인한다.
- 속도, 피치, 요 제어가 이어지는 캐스케이드 PID 구조를 구성한다.
- 엔코더 기반 속도 추정과 BLDC 3상 전압 생성을 구현한다.
- 블루투스 조이스틱 입력으로 실물 밸런싱과 수동 조작을 확인한다.
- MATLAB/Simulink HILS를 구성해 실제 ESP32 펌웨어와 가상 모델의 데이터 교환을 검증한다.
- RX-28 관절 액추에이터와 YDLIDAR G2 확장 코드를 함께 정리한다.

---

## 제어 구조

### 1. MPPI: 웨이포인트를 속도와 각속도 명령으로 바꾸는 상위 제어기

#### 역할

MPPI는 현재 위치, 목표점, 장애물 정보를 받아 여러 개의 미래 명령열을 만든다. 명령열 하나는 `v_ref`와 `w_ref`의 시간 순서이다. 각 명령열을 적용했을 때 로봇이 어디로 움직일지 예측하고, 목표점에 가까우면서 장애물을 피하는 명령열에 높은 가중치를 준다.

마지막에는 모든 후보 명령열을 가중 평균한다. 실제 로봇에는 그중 첫 번째 명령만 전달한다. 다음 제어 주기에서 다시 샘플링하므로, 로봇은 매 순간 새 센서값을 반영해 경로를 다시 고른다.

#### ESP32 펌웨어 구성

ESP32 쪽 MPPI 코드는 `SoftWare/Gnss_Mppi`에 들어 있다.

- `gnss.c`: 현재 GNSS 위치와 목표 웨이포인트를 로컬 `x/y` 좌표로 변환한다.
- `waypoint.c`: 기지국에서 받은 웨이포인트를 순서대로 목표점에 넣는다.
- `mppi.c`: 후보 명령열을 만들고, 예측 상태를 계산한 뒤 최종 `v_ref`, `w_ref`를 선택한다.
- `cost.c`: 목표점 오차, 장애물 거리, 입력 크기, 입력 변화량을 비용으로 계산한다.
- `lidar.c`: 라이다 거리 데이터를 받아 장애물 정보를 구성한다.

#### 장애물 처리

실제 로봇은 라이다 360도를 `24`개 섹터로 나눠 본다. 한 섹터 안에 장애물이 여러 개 들어오면 가장 가까운 장애물 하나만 대표값으로 저장한다. MPPI 비용 계산에서는 이 섹터 거리와 각도를 월드 좌표의 장애물 위치로 바꾼다. 예측된 로봇 위치가 안전거리 안으로 들어가면 장애물 비용이 커진다.

MATLAB 시뮬레이션에서는 장애물을 절대좌표에 직접 배치한다. 입력 방식만 다를 뿐, 예측 로봇 위치와 장애물 위치 사이의 거리로 비용을 계산한다는 점은 같다.

#### 계산 순서

1. 이전 최적 명령열을 한 칸 앞으로 당겨 현재 샘플링의 기준 명령열로 사용한다.
2. 기준 명령열에 랜덤 노이즈를 더해 후보 명령열을 만든다.
3. 후보 명령열마다 미래 로봇 위치와 방향을 예측한다.
4. 목표점 비용, 장애물 비용, 입력 비용, 입력 변화 비용을 누적한다.
5. 비용을 `exp(-(J_i - J_min) / lambda)` 형태의 MPPI 가중치로 바꾼다.
6. 가중 평균으로 최종 명령열을 만들고 첫 번째 입력만 적용한다.

#### ESP32 적용 파라미터

실제 펌웨어에서는 ESP32 계산량을 고려해 짧은 예측 구간을 사용한다.

- 제어 주기: `dt = 0.1 s`
- 예측 스텝: `horizon = 6`
- 후보 명령열: `num_samples = 64`
- 속도 범위: `-0.5 m/s` ~ `0.5 m/s`
- 각속도 범위: `-1.2 rad/s` ~ `1.2 rad/s`
- MPPI 온도 파라미터: `lambda = 10.0`
- 장애물 안전거리: `0.5 m`

#### MATLAB 사전검증

펌웨어에 넣기 전, MATLAB에서 같은 구조의 MPPI를 먼저 돌렸다. 하드웨어를 그대로 복제하려는 목적은 아니다. 목표점과 장애물이 있을 때 후보 명령열이 충분히 퍼지는지, 장애물 비용이 회피 방향에 영향을 주는지, 최종 가중 평균 궤적이 목표점 방향으로 이어지는지를 보기 위한 단계이다.

시뮬레이션 코드는 `SoftWare/Mppi_matlab_simulation`에 정리했다.

- `main.m`: 시작점, 목표점, 장애물, 반복 루프를 설정한다.
- `mppi/init_mppi_params.m`: MPPI 파라미터와 비용 가중치를 설정한다.
- `mppi/mppi_step.m`: 후보 명령열 생성, 비용 계산, 가중 평균 계산을 수행한다.
- `cost/calc_total_cost.m`: 목표점, 장애물, 입력, 부드러움 비용을 계산한다.
- `sim/draw_sim.m`: 로봇 이동 경로, 장애물, 목표점, 후보 궤적을 그린다.

[MPPI MATLAB 시뮬레이션 영상](docs/videos/mppi_test.mp4)

MATLAB 검증에서 사용한 주요 설정은 다음과 같다.

- 제어 주기: `dt = 0.1 s`
- 예측 스텝: `horizon = 30`
- 예측 시간: `3.0 s`
- 후보 명령열: `num_samples = 64`
- 속도 범위: `-0.5 m/s` ~ `0.5 m/s`
- 각속도 범위: `-1.2 rad/s` ~ `1.2 rad/s`
- MPPI 온도 파라미터: `lambda = 10.0`
- 이전 최적 명령열이 있을 때 노이즈: `v_amp = 0.30`, `w_amp = 0.80`
- 첫 탐색 노이즈: `v_amp = 0.40`, `w_amp = 0.90`

#### 비용 함수

MATLAB 비용 함수는 아래 구조로 계산한다.

```text
total_cost =
  goal_cost
  + obstacle_cost
  + input_cost
  + smooth_cost
```

검증 당시 비용 가중치는 다음과 같다.

- 목표점 x축 가중치: `weight_goal_x = 1.5`
- 목표점 y축 가중치: `weight_goal_y = 1.5`
- 장애물 비용 가중치: `weight_obstacle = 120.0`
- 속도 입력 비용 가중치: `weight_input_v = 0.1`
- 각속도 입력 비용 가중치: `weight_input_w = 0.1`
- 속도 변화 비용 가중치: `weight_smooth_v = 0.3`
- 각속도 변화 비용 가중치: `weight_smooth_w = 0.3`
- 장애물 안전거리: `obs_safe_dist = 1.0 m`

### 2. PID: MPPI 명령을 넘어지지 않는 자세 제어로 바꾸는 층

#### 역할

MPPI가 만든 `v_ref`와 `w_ref`는 모터에 바로 넣지 않는다. 밸런싱로봇은 목표 속도를 따라가면서 몸체 각도도 계속 잡아야 한다. 그래서 목표 속도는 먼저 목표 피치가 되고, 피치 제어 결과가 좌우 바퀴 전압 명령으로 이어진다.

#### 제어 흐름

```text
targetvel_vel
-> 속도 PID
-> target_pitch
-> 피치 PID
-> 균형 유지용 출력
-> yaw 보정값과 합성
-> Vq_left, Vq_right
```

속도 PID는 목표 속도와 현재 속도의 차이로 몸체가 얼마나 기울어야 하는지 정한다. 피치 PID는 현재 피치가 목표 피치에 맞도록 균형 출력을 만든다. 요 제어는 왼쪽과 오른쪽 바퀴 명령에 차이를 만들어 회전 방향을 잡는다.

#### SILS

SILS에서는 실제 하드웨어 없이 제어 응답을 먼저 본다. 응답이 발산하는지, 목표 속도나 외란에 대해 로봇 자세가 다시 안정되는지 확인한다.

<p align="center">
  <img src="docs/images/sils_test.png" alt="SILS test result" width="70%" />
</p>

#### HILS 통신 구조

SILS 이후에는 HILS를 구성한다. MATLAB/Simulink가 가상 센서 역할을 하고, 실제 ESP32 펌웨어가 제어 계산을 맡는다. 실물 로봇에 올리기 전에 데이터 패킷, 제어 주기, 반환 전압값이 의도대로 이어지는지 확인하는 단계이다.

![HILS block diagram](docs/images/hils_matlab_block.png)

PC와 ESP32는 `CP2102 USB-to-TTL` 모듈로 연결한다. ESP32는 `UART2`를 사용하고, `TX=GPIO17`, `RX=GPIO18`에 연결한다.

MATLAB에서 ESP32로 보내는 값은 오른쪽 엔코더, 왼쪽 엔코더, pitch, yaw이다. 각 값은 `single(float)`이므로 4바이트이다. 한 번 보낼 때 `4개 * 4바이트 = 16바이트`가 된다.

ESP32는 받은 센서값으로 제어를 계산하고, 왼쪽과 오른쪽 BLDC의 3상 전압을 MATLAB으로 돌려보낸다. 반환값은 총 6개 float이므로 `6개 * 4바이트 = 24바이트`이다. Simulink에서는 `Serial Receive`로 24바이트를 받고, `Byte Unpack`으로 6개의 `single(float)` 값으로 복원한다.

#### 보레이트 계산

[HILS 구조 참고 PDF](docs/references/hils_flow.pdf) 기준으로 MATLAB에서 ESP32로 보내는 센서 패킷은 1kHz 주기로 들어온다.

```text
16 byte/packet * 1000 packet/s = 16000 byte/s
```

UART는 일반적으로 8비트 데이터 앞뒤에 start bit와 stop bit가 붙는다. 즉 1바이트를 보내려면 실제 라인에서는 10비트가 필요하다.

```text
16000 byte/s * 10 bit/byte = 160000 bit/s
```

ESP32에서 MATLAB으로 돌아오는 값은 24바이트 패킷이다. 같은 1kHz 기준으로 보면 아래와 같다.

```text
24 byte/packet * 1000 packet/s * 10 bit/byte = 240000 bit/s
```

양방향 모두 같은 UART 설정을 쓰므로 더 큰 쪽인 `240000 bps`보다 높은 보레이트가 필요하다. 그래서 `921600 bps`를 사용했다. 계산상 필요한 속도보다 약 3.8배 여유가 있고, HILS에서 1kHz 데이터 교환을 할 때 패킷 지연 여지를 줄일 수 있다.

#### 속도 추정

엔코더 각도는 `1 kHz`로 읽지만 속도 추정은 매 주기마다 갱신하지 않는다. `1 ms` 차분에서는 각도 변화량이 너무 작다. 작은 엔코더 흔들림도 큰 속도 스파이크처럼 계산될 수 있다.

그래서 엔코더 각도는 계속 `1 kHz`로 샘플링하고, 속도 추정만 `10`회마다 한 번 갱신한다. 실제 속도 추정 주기는 `10 ms`이다. `+/-pi` 경계에서 각도 차이가 튀지 않도록 언랩 처리를 넣고, `0.5 / 0.5` 저역통과 필터로 남은 노이즈를 줄인다.

#### RTOS 스케줄링

실물 로봇에서는 센서 요청과 모터 출력의 타이밍이 바로 진동으로 이어졌다. 엔코더 경로는 GPTimer `1 MHz` 해상도와 `alarm_count = 1000` 설정으로 `1 ms`마다 동작한다. SPI 엔코더 읽기가 끝나면 `spi_post_callback()`에서 `encoder_sem`을 깨운다. `motor_control_task`는 새 엔코더 값이 준비됐을 때만 모터 전압 계산을 수행한다.

IMU 경로는 `IMU_ALARM_COUNT = 5000`으로 `5 ms`, 즉 `200 Hz`마다 동작한다. 모터 제어 태스크는 우선순위 `5`, RX-28 태스크는 우선순위 `4`로 분리한다. 엔코더 기반 모터 출력 경로를 더 높은 우선순위로 두면서 균형 제어의 시간 흔들림을 줄인다.

#### 조이스틱 입력

블루투스 조이스틱 입력은 자동 수색 미션의 주 경로가 아니라 실물 밸런싱 테스트용 입력이다. `hc06.c`는 `S,x,y,diff,E` 형식의 패킷을 파싱한다. 조이스틱이 중앙 근처에 있으면 목표 속도와 요 명령을 0으로 만든다. 일정 크기 이상 움직이면 전진 속도와 회전 명령을 생성한다.

이 입력으로 실물 로봇이 넘어지지 않고 전진, 정지, 회전을 수행하는지 확인한다.

#### RX-28 관절 제어

RX-28 관절 제어는 카메라 흔들림을 줄이기 위한 확장 구조이다. IMU에서 측정한 `roll` 값을 기준으로 보정 길이 `roll_adj_mm`를 만든다. 좌우 다리 높이를 다르게 계산해 골반과 무릎 RX-28 목표 위치로 변환한다.

`rx28.c`는 미리 계산한 다리 높이 테이블을 이용해 목표 높이에 맞는 관절 값을 보간한다. 바퀴 제어만으로 처리하기 어려운 좌우 기울어짐을 관절 쪽에서 보정하는 구조이다.

### 3. DQ 변환: Vq 명령을 BLDC 3상 전압으로 바꾸는 층

#### 역할

PID를 거쳐 나온 최종 명령은 왼쪽과 오른쪽 바퀴의 `Vq_left`, `Vq_right`이다. 이 값은 BLDC 모터의 q축 전압 명령이다. 실제 모터에는 3상 전압 `Va`, `Vb`, `Vc`가 들어가야 한다. 그래서 엔코더 각도를 이용해 역 Park 변환과 Clarke 변환을 수행한다.

#### SPI 타이밍 문제

DQ 변환에서 가장 먼저 맞춰야 했던 값은 엔코더 각도이다. AS5048A 엔코더는 SPI로 16비트 데이터를 보낸다. 이때 CSn을 낮춘 뒤 첫 번째 클럭 상승까지 필요한 최소 시간이 데이터시트 기준 `tL = 350 ns`이다.

<p align="center">
  <img src="docs/images/spi_timing.png" alt="AS5048A SPI timing" width="70%" />
</p>

초기에는 CSn을 낮춘 직후 바로 데이터를 읽으면서 16비트 저장 위치가 한 칸씩 밀렸다. 첫 번째 클럭이 아직 생성되기 전인데 MCU가 `data[15]` 자리를 읽으려 하면, 그 위치에는 유효한 센서 데이터가 들어오지 않는다. 결과적으로 15번째 비트에는 의미 없는 값이 들어가고, 14번째 비트에는 원래 `data15`, 13번째 비트에는 원래 `data14`가 들어가는 식으로 데이터가 오른쪽으로 밀린다.

MCU는 하위 14비트를 각도 원시값으로 해석한다. 당시에는 에러 비트가 항상 1처럼 보였고, 밀린 데이터 때문에 각도 원시값의 상위 비트가 계속 켜지는 현상이 발생했다. 그 결과 각도값에 180도가 더해진 것처럼 해석되어 `180~360 deg` 구간 위주로 값이 나왔다.

해결은 CSn 이후 첫 클럭까지의 대기 시간을 SPI 설정으로 보장하는 방식이다. 현재 엔코더 SPI 클럭은 `10 MHz`이고, 한 클럭은 `100 ns`이다. `encoder.c`에서 `.cs_ena_pretrans = 4`로 설정해 약 `400 ns`를 기다린다. 이 값은 필요한 `350 ns`보다 길기 때문에 첫 번째 클럭 전에 데이터가 밀리는 문제를 줄인다.

#### 전기각 오프셋

엔코더 각도는 전기각 오프셋도 맞춰야 한다. `d = 상수`, `q = 0`으로 두면 토크를 만드는 q축 명령은 0이 된다. 대신 회전자를 고정시키는 d축 성분이 상수로 들어가므로 회전자가 한 위치에 고정된다. 이때의 엔코더 값은 전기적 각도 0도에 해당하는 기계적 기준값이다. 이 값이 오프셋이다.

엔코더가 그 순간 0을 읽는 것은 아니다. 그래서 실제 제어에서는 읽어온 엔코더 각도에서 오프셋 각도를 뺀 뒤 극쌍수를 곱해 전기각을 계산한다.

```text
electrical_angle = (measured_encoder_angle - offset_angle) * pole_pairs
```

현재 코드에서는 `GM4108H-120T` 모터의 극쌍수 `11`을 사용한다. 왼쪽 바퀴 오프셋은 `2.7756 rad`, 오른쪽 바퀴 오프셋은 `1.6345 rad`이다.

#### 3상 전압 생성

전기각을 계산한 뒤 `Vd = 0`, `Vq = Vq_left/right`를 기준으로 역 Park 변환을 수행한다. 이후 Clarke 변환으로 `Va`, `Vb`, `Vc`를 만든다. 이 3상 전압이 MCPWM 출력으로 전달되어 BLDC 모터를 구동한다.

---

## 실물 테스트

HILS 이후에는 실물 로봇으로 외란 복귀, 위치 유지, 블루투스 주행을 확인한다. 이 단계에서는 제어 알고리즘 자체뿐 아니라 태스크 우선순위, 엔코더 속도 추정 주기, 조이스틱 입력이 실제 균형 유지에 어떤 영향을 주는지 함께 확인한다.

<p align="center">
  <img src="docs/images/real_robot.png" alt="real robot" width="45%" />
  <img src="docs/images/sim_robot.png" alt="simulation robot" width="45%" />
</p>

실물 테스트에서 중점적으로 보는 항목은 다음과 같다.

- 외부에서 밀었을 때 로봇이 다시 자세를 회복하는지 확인한다.
- 목표 속도를 줄 때 몸체가 자연스럽게 기울어지며 따라가는지 확인한다.
- 블루투스 조이스틱으로 전진, 정지, 회전을 입력해도 밸런싱이 깨지지 않는지 확인한다.

---

## 검증 자료

GitHub README에서는 mp4가 항상 바로 재생되지 않기 때문에 영상은 링크로 정리한다.

- [MPPI MATLAB 시뮬레이션](docs/videos/mppi_test.mp4)
- [HILS 테스트 영상](docs/videos/hils_test.mp4)
- [SILS yaw 테스트 영상](docs/videos/sils_yaw_test.mp4)
- [SILS 목표 속도 테스트 영상](docs/videos/sils_target_velocity_test.mp4)
- [SILS 외란 복귀 테스트](docs/videos/sils_disturbance_rejection.mp4)
- [실물 외란 복귀 테스트](docs/videos/real_world_disturbance_rejection.mp4)
- [실물 위치 유지 테스트](docs/videos/real_world_hold_position_after_disturbance.mp4)
- [실물 블루투스 조작 테스트](docs/videos/real_world_bluetooth_control.mp4)
- [BLDC 역 DQ 제어 영상 Q=1](docs/videos/inverse_dq_control_q1.mp4)
- [BLDC 역 DQ 제어 영상 Q=4](docs/videos/inverse_dq_control_q4.mp4)
- [HILS 구조 참고 PDF](docs/references/hils_flow.pdf)
- [밸런싱로봇 정리 PDF](docs/images/balancing_robot_evidence.pdf)

---

## 하드웨어 구성

최종 실물 테스트 기준 주요 부품은 다음과 같다.

- 메인 컨트롤러: `ESP32-S3-WROOM-1 N16R8`
- 휠 모터: `GM4108H-120T` BLDC 모터 2개
- 휠 모터 드라이버: `MKS SimpleFOC MINI` 2개
- 휠 엔코더: `AS5048A` 2개
- 관절 액추에이터: `Dynamixel RX-28` 4개
- 통신 변환기: `MAX485 TTL to RS-485`
- IMU: `WT901`
- 블루투스 모듈: `HC-06`
- HILS UART 브리지: `CP2102 USB-to-TTL`
- 전원: `3S LiPo battery`, `XL6009` 승압 모듈

검토 또는 확장용으로 사용한 부품은 다음과 같다.

- `U2D2`
- `U2D2 Power Hub`
- `YDLIDAR G2`
- `MPU6050`

최종 밸런싱 테스트에서 실제로 사용한 IMU는 `WT901`이다.

---

## 파일 구조

- `SoftWare/Physical_operation_code/`: 실물 밸런싱로봇용 ESP-IDF 펌웨어이다.
- `SoftWare/Physical_operation_code/app_main.c`: 태스크 생성, 세마포어 대기, PID 제어 통합을 담당한다.
- `SoftWare/Physical_operation_code/encoder.c`: 엔코더 SPI 읽기, 속도 추정, 전기각 계산, 역 DQ 변환, 3상 전압 출력을 담당한다.
- `SoftWare/Physical_operation_code/imu.c`: WT901 IMU 값을 읽고 200 Hz 상태 갱신 경로를 구성한다.
- `SoftWare/Physical_operation_code/pid.c`: 속도, 피치, 요, 롤 제어에 사용하는 PID 계산을 담당한다.
- `SoftWare/Physical_operation_code/pwm.c`: MCPWM 기반 모터 전압 출력을 담당한다.
- `SoftWare/Physical_operation_code/hc06.c`: 블루투스 조이스틱 입력을 파싱한다.
- `SoftWare/Physical_operation_code/rx28.c`: RX-28 관절 제어와 roll 기반 보정 구조를 담당한다.
- `SoftWare/Physical_operation_code/lidar.c`: YDLIDAR G2 확장 코드를 포함한다.
- `SoftWare/Gnss_Mppi/`: GNSS 웨이포인트, 24섹터 라이다 장애물, MPPI 명령 생성이 포함된 펌웨어 변형이다.
- `SoftWare/Mppi_matlab_simulation/`: ESP32 MPPI 적용 전 MATLAB에서 명령 생성 로직을 검증한 코드이다.
- `SoftWare/hils_test_code/`: MATLAB/Simulink HILS 검증용 ESP-IDF 프로젝트이다.

---

## 빌드 방법

이 프로젝트는 `ESP-IDF` 기준으로 작성한다.

실물 로봇 코드는 `SoftWare/Physical_operation_code`에서 빌드한다.

```bash
idf.py build
```

보드에 플래시한다.

```bash
idf.py -p (PORT) flash
```

필요하면 시리얼 모니터를 연다.

```bash
idf.py -p (PORT) monitor
```

HILS용 코드를 사용할 때는 `SoftWare/hils_test_code`로 이동한 뒤 같은 `idf.py` 흐름을 사용한다. GNSS와 MPPI가 포함된 자율 주행용 코드는 `SoftWare/Gnss_Mppi`에서 확인한다.

---

## 한 줄 요약

GPS 웨이포인트 기반 수색 로봇을 만들기 위해 MPPI로 상위 이동 명령을 만들고, PID로 균형을 유지하며, 엔코더 보정 기반 역 DQ 변환으로 BLDC 모터를 구동한 ESP32 밸런싱로봇 프로젝트이다.
