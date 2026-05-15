# ESP32 Self-Balancing Robot with HILS, SILS, and Real-World Validation

This repository contains the ESP-IDF firmware for a two-wheeled self-balancing robot. The project was validated in three stages: HILS (Hardware-in-the-Loop Simulation) for communication and control-chain verification, SILS (Software-in-the-Loop Simulation) for controller behavior checks, and physical-world testing for disturbance rejection and Bluetooth driving.

---

## Features

- **Cascaded balance control**: The firmware converts target velocity into target pitch, then converts target pitch into motor q-axis voltage for upright balancing.
- **Differential yaw control**: Steering is injected as a bounded left/right voltage difference without breaking the main balance loop.
- **Real-time sensing and actuation**: The IMU loop runs at 200 Hz while the encoder and motor commutation path runs at 1 kHz.
- **MATLAB/Simulink HILS workflow**: The project includes a documented serial-based HILS pipeline for testing the control chain before full hardware deployment.
- **Wireless command input**: An HC-06 Bluetooth module is used to receive joystick commands and translate them into velocity and yaw references.
- **Exact hardware stack**: The final build uses dual GM4108H-120T BLDC wheel motors, dual MKS SimpleFOC MINI driver boards, four Dynamixel RX-28 actuators, an ESP32-S3-WROOM-1 N16R8 controller, WT901 IMU, AS5048A magnetic encoders, MAX485, XL6009 converters, and a 3S LiPo battery.

---

## System Design & Control Logic

### 1. Main balance loop

The control loop is split across two timing domains. `imu_timer_init()` starts the 200 Hz IMU update cycle, and `imu_data_cal()` refreshes `current_pitch`, `current_yaw`, and `current_roll`. In parallel, `encoder_timer_init()` starts the 1 kHz encoder path, and `motor_control_task()` wakes on the encoder semaphore to execute `encoder_to_vcc_cal()`.

Inside `app_main.c`, the outer velocity loop converts `targetvel_vel` into `target_pitch`. That target is smoothed before being passed into the pitch loop, which reduces oscillation when speed commands change. A separate yaw command is turned into a bounded differential term, then mixed with the pitch output to produce `Vq_left` and `Vq_right`.

Inside `encoder.c`, the motor-side loop reconstructs left and right encoder angles, estimates wheel speed, converts the q-axis voltages into electrical angles, and finally generates six phase voltages for the two BLDC motors. Those voltages are applied through `mcpwm_set_voltage()` in `pwm.c`.

### 2. Command input path

`hc06.c` receives joystick packets in the `S,x,y,diff,E` format over UART2. The parser recenters the joystick coordinates, suppresses small steering noise with a deadband, and maps the joystick magnitude to either a stop command or a forward velocity target. This keeps wireless control simple while still exercising the real balance and steering logic.

### 3. Additional hardware support

`rx28.c` runs a dedicated FreeRTOS task for four Dynamixel RX-28 actuators through a MAX485 TTL-to-RS-485 link. `lidar.c` contains experimental YDLIDAR G2 support, and `hc06.c` handles Bluetooth joystick commands over UART. `variable.h` also already reserves HILS-oriented shared variables such as `hils_angle_l`, `hils_angle_r`, `hils_pitch`, `hils_yaw`, and `hils_v_out[6]`, making it easier to extend the MATLAB integration further.

---

## Demonstration Videos

GitHub does not reliably inline-play repository `mp4` files inside `README.md`, so the videos below are linked directly.

### 1. HILS test

[Open HILS test video](assets/videos/hils_test.mp4)

[HILS communication flow reference](assets/docs/hils_flow.pdf)

This video shows the control pipeline running in a Hardware-in-the-Loop setup before applying the algorithm to the physical robot. According to the HILS flow document, MATLAB/Simulink sends four `single` values every 1 ms: left encoder angle, right encoder angle, pitch, and yaw. That becomes a 16-byte packet from MATLAB to ESP32. The same document explains why `921600 bps` was used for the serial link: even after accounting for UART start and stop bits, the receive side only needs about `160 kbps`, so the link has enough margin for stable round-trip communication.

The flow document also shows the physical HILS wiring: a CP2102 USB-to-TTL module is connected to ESP32 UART2 on pins TX17 and RX18. On the control side, the HILS loop mirrors the real firmware structure. The 200 Hz balance update generates `Vq_left` and `Vq_right` from the cascaded velocity and pitch logic, and the 1 kHz motor-side loop expands those values into six phase voltages for the left and right BLDC motors. The HILS return path is described as six `float` phase voltages packed into a 24-byte packet, which matches the six-voltage structure produced by the firmware in `encoder.c`.

### 2. SILS yaw test

[Open SILS yaw test video](assets/videos/sils_yaw_test.mp4)

This video isolates the yaw branch of the controller. In `app_main.c`, the yaw command does not replace the balance loop. Instead, it is converted into a bounded turning term and added to the left motor while being subtracted from the right motor. Because steering is implemented as a differential overlay on top of the pitch output, the test is useful for checking whether the robot can rotate without collapsing the balance controller.

### 3. SILS target velocity test

[Open SILS target velocity test video](assets/videos/sils_target_velocity_test.mp4)

This test focuses on the outer velocity loop. In `encoder.c`, wheel speed is estimated from encoder angle differences every 10 ms, angle wraparound is corrected, and a light low-pass filter is applied before the result is stored in `current_vel`. When a fresh speed estimate is ready, `app_main.c` uses `vel_ctrl` to convert target velocity into target pitch and smooths that pitch target once more before the balance loop uses it. The video shows that the controller responds to a speed command by changing body lean first instead of applying abrupt torque changes.

### 4. Physical world: external disturbance rejection

[Open physical-world disturbance rejection video](assets/videos/real_world_disturbance_rejection.mp4)

This video demonstrates short-push recovery in the real robot. The recovery behavior comes from the fast pitch loop in `app_main.c` together with the 1 kHz encoder-driven motor update in `encoder.c`. Because the firmware keeps recalculating motor phase voltages from the updated chassis state, the robot can reject brief disturbances and recover to an upright posture quickly.

### 5. Physical world: external disturbance hold position

[Open physical-world hold-position video](assets/videos/real_world_hold_position_after_disturbance.mp4)

This test is slightly different from simple fall recovery. Here the important point is that the robot reduces residual drift after being disturbed. When `targetvel_vel` is zero, the velocity feedback path works to pull the motion back toward a stationary state, while the pitch loop keeps the chassis upright during the same recovery sequence.

### 6. Physical world: Bluetooth teleoperation

[Open physical-world Bluetooth teleoperation video](assets/videos/real_world_bluetooth_control.mp4)

This video maps directly to the HC-06 command parser in `hc06.c`. The firmware reads packets in the `S,x,y,diff,E` format, converts joystick coordinates into centered values, removes small steering noise with deadbands, and maps larger joystick magnitude into a forward velocity command. `app_main.c` then combines that velocity target with yaw mixing to generate separate left and right motor voltages, allowing the robot to drive and steer without giving up self-balancing behavior.

---

## Hardware Components

This project uses the following modules in the final hardware stack:

- **Main controller**: ESP32-S3-WROOM-1 N16R8
- **Wheel motors**: GM4108H-120T BLDC motors x2
- **Wheel motor drivers**: MKS SimpleFOC MINI BLDC motor driver boards x2
- **Wheel encoders**: AS5048A magnetic SPI encoders x2
- **Leg actuators**: Dynamixel RX-28 x4
- **Servo bus interface**: MAX485 TTL-to-RS-485 converter module
- **IMU sensor**: WT901
- **Wireless control**: HC-06 Bluetooth UART module
- **HILS serial bridge**: CP2102 USB-to-TTL module
- **Power**: 3S LiPo battery and XL6009 boost converters

Additional lab and expansion modules referenced in the project:

- **Dynamixel test interface**: U2D2 and U2D2 Power Hub
- **Optional ranging module**: YDLIDAR G2 support exists in firmware
- **Not used in final build**: MPU6050 was evaluated but replaced by WT901

---

## Software & Setup

This firmware is developed using the Espressif IoT Development Framework (ESP-IDF).

### Prerequisites

- A working installation of the ESP-IDF toolchain
- For HILS, a MATLAB/Simulink environment configured for serial communication

### Build and flash

1. Navigate to the project root directory.
2. Open an ESP-IDF command prompt.
3. Build the project:

```bash
idf.py build
```

4. Flash the firmware to the ESP32:

```bash
idf.py -p (PORT) flash
```

5. Open the serial monitor if needed:

```bash
idf.py -p (PORT) monitor
```

---

## Project Structure

| File | Description |
| :--- | :--- |
| `app_main.c` | Main application entry point. Initializes the control loops, tasks, and shared state. |
| `pid.c` / `pid.h` | PID controller implementation used by the velocity and pitch loops. |
| `imu.c` / `imu.h` | WT901 initialization, timer scheduling, and attitude data acquisition. |
| `encoder.c` / `encoder.h` | Encoder acquisition, velocity estimation, and motor phase-voltage generation. |
| `pwm.c` / `pwm.h` | Low-level MCPWM voltage application for the six motor phases. |
| `hc06.c` / `hc06.h` | Bluetooth command input and parsing logic. |
| `rx28.c` / `rx28.h` | RX-28 leg actuator control task and interpolation logic. |
| `lidar.c` / `lidar.h` | LiDAR UART interface for future sensing extensions. |
| `variable.h` | Shared constants, globals, and cross-module control variables. |
