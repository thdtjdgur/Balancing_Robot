# ESP32 기반 밸런싱 로봇 프로젝트

이 저장소는 내가 직접 만든 2륜 밸런싱 로봇의 ESP-IDF 펌웨어 코드다.  
처음부터 실기체만으로 제어기를 맞추면 시간도 오래 걸리고 넘어질 때 리스크도 커서, 이번 프로젝트는 `SILS -> HILS -> 실제 로봇 테스트` 순서로 검증하면서 제어기를 다듬었다.

로봇 구조는 단순히 바퀴만 있는 형태가 아니라, `GM4108H-120T` BLDC 휠 모터와 `RX-28` 관절부를 같이 사용하는 형태다. 그래서 이 README도 "코드가 무슨 역할을 하는지", "왜 HILS를 넣었는지", "실제로 어떤 방식으로 검증했는지" 중심으로 정리했다.

---

## 프로젝트에서 구현한 것

- ESP32-S3에서 돌아가는 밸런싱 제어 루프 구현
- `속도 -> 목표 pitch -> pitch 제어 -> 좌우 모터 전압 분배` 구조의 제어기 구성
- 엔코더 기반 속도 추정과 3상 전압 생성
- Bluetooth 조이스틱 입력으로 전진/정지/회전 명령 처리
- MATLAB/Simulink와 UART로 연결하는 HILS 검증 환경 구성
- SILS, HILS, 실제 로봇 테스트까지 단계별 검증

---

## 제어 구조

이 프로젝트의 핵심은 "넘어지지 않게 자세를 잡는 것"과 "원하는 방향으로 움직이게 만드는 것"을 분리해서 제어한 점이다.

### 1. 메인 밸런싱 루프

- `imu_timer_init()`으로 IMU 쪽은 `200 Hz`
- `encoder_timer_init()`으로 엔코더 쪽은 `1 kHz`
- IMU에서는 `pitch`, `yaw`, `roll` 상태를 갱신
- 엔코더 쪽에서는 바퀴 각도와 속도를 계산하고 실제 모터에 들어갈 전압을 만듦

코드 흐름은 대략 이렇게 간다.

1. 목표 속도를 바로 모터에 넣지 않는다.
2. 먼저 속도 오차를 이용해서 `목표 pitch`를 만든다.
3. 그다음 현재 pitch와 목표 pitch 차이로 밸런싱 출력을 만든다.
4. 회전은 yaw 성분을 좌우 전압 차이로 섞어서 처리한다.
5. 최종적으로 `Vq_left`, `Vq_right`를 만들고, 이 값을 3상 전압으로 바꿔 BLDC 모터에 넣는다.

즉 이 프로젝트는 "속도 제어기가 몸을 어느 정도 기울일지 결정하고, pitch 제어기가 실제로 안 넘어지게 받쳐주는 구조"라고 보면 된다.

### 2. 조이스틱 입력 처리

`hc06.c`에서는 Bluetooth로 들어오는 조이스틱 값을 받아서 `S,x,y,diff,E` 형식으로 파싱한다.

- 작은 입력은 deadband로 무시
- 조이스틱 크기가 작으면 정지
- 일정 이상 밀면 전진 목표 속도 부여
- 회전은 좌우 전압 차이로 분배

그래서 실제 주행에서는 "앞으로 가는 동작"과 "제자리 회전/방향 전환"이 완전히 따로 노는 게 아니라, 밸런싱 위에 자연스럽게 얹히는 방식으로 동작한다.

### 3. 관절부와 확장 하드웨어

이 로봇은 바퀴만 제어하는 게 아니라 관절부도 같이 들어간다.

- `rx28.c`에서 `RX-28` 4개 제어
- `MAX485`를 통해 TTL-RS485 변환
- `lidar.c`에는 `YDLIDAR G2` 확장 코드 포함

현재 README의 중심은 밸런싱 파트지만, 코드 구조는 이후 센서나 관절 제어를 더 얹을 수 있게 나눠둔 상태다.

---

## 검증 과정

이번 프로젝트는 바로 실기체만 붙잡고 튜닝한 게 아니라, 아래 순서대로 검증했다.

### 1. SILS

먼저 제어기가 이론상 제대로 반응하는지 SILS에서 확인했다.

<p align="center">
  <img src="docs/images/sils_test.png" alt="SILS test result" width="70%" />
</p>

여기서는 제어기 응답이 아예 말이 안 되는 수준은 아닌지, 목표값이 들어왔을 때 발산하지 않는지, 밸런싱이 가능한 형태로 수렴하는지를 먼저 봤다. 실기체 튜닝 전에 가장 먼저 확인한 단계다.

### 2. HILS

SILS 다음에는 실제 ESP32 코드가 MATLAB/Simulink 모델과 제대로 데이터를 주고받는지 확인하려고 HILS를 구성했다.

![HILS block diagram](docs/images/hils_matlab_block.png)

HILS에서 중요한 건 "시뮬레이션만 맞는 게 아니라, 실제 펌웨어가 같은 제어 흐름으로 돌아가느냐"였다.  
그래서 PC와 ESP32를 `CP2102 USB-to-TTL` 모듈로 연결하고, ESP32 `UART2`를 사용했다.

- ESP32 TX: `GPIO17`
- ESP32 RX: `GPIO18`
- PC-ESP32 사이 직렬 통신 브리지: `CP2102`

#### MATLAB -> ESP32

`hils_flow.pdf` 기준으로 MATLAB 쪽에서 보내는 데이터는 총 4개다.

- 오른쪽 엔코더값
- 왼쪽 엔코더값
- pitch
- yaw

각 값은 `single(float)` 1개씩이고, float 1개가 4바이트라서 한 번에 보내는 패킷 크기는 `16 bytes`다.

문서 기준으로 보면

- 엔코더값은 `1 kHz`
- pitch, yaw는 `200 Hz`

로 갱신된다.  
실제로는 MATLAB에서 이 값들을 `Byte Pack`으로 묶어서 UART로 보내고, ESP32에서는 그 데이터를 받아 제어기에 넣는다.

전송량도 계산해 보면

- `16 bytes * 1000 = 16000 bytes/s`
- UART는 start/stop bit가 붙으니까 실제 필요한 선속도는 더 커짐
- 문서 기준 대략 `160000 bps`

정도가 필요하다. 그래서 이 프로젝트에서는 `921600 bps`를 사용해서 통신 여유를 충분히 둔 상태로 HILS를 돌렸다.

#### ESP32 -> MATLAB

ESP32 쪽에서는 받은 엔코더값과 자세값을 바탕으로 제어기를 돌린 다음, 최종적으로 좌우 BLDC에 들어갈 3상 전압을 만들어 MATLAB로 다시 넘긴다.

왼쪽 3상 + 오른쪽 3상이라서 총 `6개 float`가 나가고,

- `6 floats * 4 bytes = 24 bytes`

가 된다.

즉 HILS에서는

- MATLAB이 센서 역할을 해서 `16바이트` 상태 패킷을 ESP32로 보내고
- ESP32는 계산한 `24바이트` 3상 전압 패킷을 MATLAB로 다시 보내는 구조

로 검증한 셈이다.

MATLAB에서는 `Serial Receive`로 24바이트를 받은 뒤 `Byte Unpack`으로 다시 6개의 float로 복원해서 가상 BLDC 모델에 넣었다.

이 단계 덕분에 실제 하드웨어를 계속 넘어뜨리지 않고도, "ESP32 코드가 실제 제어 흐름대로 돌아가는지"를 꽤 안전하게 확인할 수 있었다.

### 3. 실제 로봇 테스트

HILS까지 확인한 다음에는 실제 로봇에서 외력 대응, 속도 목표 추종, Bluetooth 주행까지 확인했다.

<p align="center">
  <img src="docs/images/real_robot.png" alt="real robot" width="45%" />
  <img src="docs/images/sim_robot.png" alt="simulation robot" width="45%" />
</p>

실제 테스트에서는 특히 아래를 중점적으로 봤다.

- 외력을 줬을 때 다시 자세를 회복하는지
- 목표 속도에 따라 몸을 자연스럽게 기울이며 움직이는지
- Bluetooth 조작 중에도 밸런싱이 깨지지 않는지

---

## 영상 자료

GitHub README에서는 저장소 안 `mp4`가 인라인으로 깔끔하게 재생되지 않는 경우가 있어서, 영상은 링크로 정리했다.

- [HILS 테스트 영상](docs/videos/hils_test.mp4)
- [SILS yaw 테스트 영상](docs/videos/sils_yaw_test.mp4)
- [SILS 목표속도 테스트 영상](docs/videos/sils_target_velocity_test.mp4)
- [실물 외력 저항 테스트](docs/videos/real_world_disturbance_rejection.mp4)
- [실물 외력 후 제자리 유지 테스트](docs/videos/real_world_hold_position_after_disturbance.mp4)
- [실물 Bluetooth 주행 테스트](docs/videos/real_world_bluetooth_control.mp4)
- [HILS 구조 설명 PDF](docs/references/hils_flow.pdf)

추가 자료:

- [밸런싱 로봇 정리 PDF](docs/images/balancing_robot_evidence.pdf)

---

## 최종 사용 부품

이번 프로젝트에서 최종적으로 사용한 부품은 아래와 같다.

- 메인 보드: `ESP32-S3-WROOM-1 N16R8`
- 휠 모터: `GM4108H-120T` BLDC 모터 2개
- 휠 모터 드라이버: `MKS SimpleFOC MINI` 2개
- 휠 엔코더: `AS5048A` 2개
- 관절 액추에이터: `Dynamixel RX-28` 4개
- 통신 변환: `MAX485 TTL to RS-485`
- IMU: `WT901`
- Bluetooth: `HC-06`
- HILS용 UART 브리지: `CP2102 USB-to-TTL`
- 전원: `3S LiPo Battery`, `XL6009` 승압 컨버터

추가로 테스트/확장용으로 확인했던 항목:

- `U2D2`, `U2D2 Power Hub`
- `YDLIDAR G2`
- `MPU6050`

다만 최종 밸런싱 테스트 기준으로는 `WT901`을 실제 사용했다.

---

## 폴더/파일 설명

- `app_main.c`: 전체 제어 흐름 시작점
- `encoder.c`, `encoder.h`: 엔코더 읽기, 속도 계산, 3상 전압 생성
- `imu.c`, `imu.h`: WT901 데이터 읽기
- `pid.c`, `pid.h`: PID 계산
- `pwm.c`, `pwm.h`: 실제 PWM 출력
- `hc06.c`: Bluetooth 조이스틱 입력 처리
- `rx28.c`, `rx28.h`: RX-28 제어
- `lidar.c`, `lidar.h`: LiDAR 확장 코드
- `variable.h`: 전역 제어 변수와 상수

---

## 빌드 방법

이 프로젝트는 `ESP-IDF` 기준으로 작성했다.

1. 프로젝트 폴더로 이동
2. ESP-IDF 환경 실행
3. 빌드

```bash
idf.py build
```

4. 보드에 업로드

```bash
idf.py -p (PORT) flash
```

5. 시리얼 모니터 확인

```bash
idf.py -p (PORT) monitor
```

---

## 한 줄 정리

이 프로젝트는 "실제로 넘어지지 않는 밸런싱 로봇을 만들기 위해, ESP32 제어 코드와 MATLAB/Simulink 검증 환경을 같이 구성하고, SILS -> HILS -> 실기 테스트 순서로 제어기를 맞춘 프로젝트"라고 보면 된다.
