# ESP32 Self-Balancing Robot

This repository contains the ESP-IDF firmware I wrote for my two-wheeled self-balancing robot.

I did not tune this project by jumping straight into full hardware tests. I verified the controller in three steps:

- `SILS` to check whether the controller logic behaved as expected
- `HILS` to check whether the actual ESP32 firmware and the Simulink model exchanged data correctly
- `real robot testing` to confirm disturbance recovery, speed response, and Bluetooth driving on hardware

This robot is not just a pair of wheels. I used `GM4108H-120T` BLDC wheel motors together with `RX-28` joint actuators, so I wanted this README to explain not only what the code does, but also how I validated the controller before relying on the real machine.

The current navigation target for this project is a waypoint-based search mission. A ground station sends GPS waypoint data, the ESP32 converts that mission into local `x/y` targets, and the robot moves through the waypoints in order while avoiding nearby obstacles. The high-level command generator is an `MPPI` controller: it samples candidate `v_ref` and `w_ref` command sequences, predicts the robot motion, scores each sequence with a cost function, and applies the first command from the weighted best sequence.

At the lower level, those MPPI commands are not sent directly to the motors. The velocity and yaw targets pass through the balancing control stack that I had already validated through `SILS` and `HILS`, and the final wheel-side commands are converted into the `Vq_left` and `Vq_right` values used by the BLDC 3-phase voltage generation path.

---

## What I built

- An ESP32-S3 balancing control loop
- A GNSS waypoint mission layer for moving through ground-station targets in sequence
- An MPPI command generator that produces target velocity and angular velocity
- A MATLAB MPPI simulation used before integrating the planner into the ESP32 code
- A cascaded controller with the structure `velocity -> target pitch -> pitch control -> left/right motor voltage split`
- Encoder-based velocity estimation and 3-phase voltage generation for the BLDC motors
- Bluetooth joystick control for forward motion, stop, and turning
- A MATLAB/Simulink HILS setup connected over UART
- A validation flow that went from SILS to HILS and then to the physical robot

---

## Control architecture

The main idea in this project was to separate "keeping the robot upright" from "making it move where I want."

At mission level, the stack is:

```text
Ground-station GPS waypoints
-> GNSS/local x-y conversion
-> waypoint manager
-> MPPI planner: v_ref, w_ref
-> velocity/yaw/pitch PID loops
-> Vq_left, Vq_right
-> BLDC inverse dq / 3-phase voltage output
```

### 1. Waypoint and MPPI command layer

The autonomous navigation layer is implemented around `Gnss_Mppi`.

- `gnss.c` stores the robot position as local `gnss_x`, `gnss_y` and converts waypoint data into local `goal_x`, `goal_y` coordinates.
- `waypoint.c` advances through the waypoint list and marks the mission complete after the final waypoint is reached.
- `mppi.c` reads the current state `[x, y, psi, v, w]` and samples candidate input sequences made of `[v_ref, w_ref]`.
- `cost.c` scores each predicted trajectory with goal distance, obstacle distance, input size, and input smoothness costs.
- The selected command updates `targetvel_vel` and `target_yaw_diff`, which are then followed by the existing balancing controller.

The obstacle side uses a compact LiDAR representation. The firmware divides the 360-degree scan into `24` sectors and keeps the nearest obstacle in each sector. During cost evaluation, that sector distance and angle are converted into a world-coordinate obstacle point, and each predicted robot state is penalized if it enters the configured safe distance.

The MPPI update itself follows the same receding-horizon pattern used in the MATLAB validation:

1. Shift the previous best command sequence forward.
2. Sample candidate command sequences around that base sequence.
3. Predict future robot states for every candidate.
4. Accumulate the trajectory cost.
5. Convert costs into exponential weights with `lambda`.
6. Compute the weighted command sequence and apply only the first input.

### 2. Main balancing loop

- `imu_timer_init()` runs the IMU-side update at `200 Hz`
- `encoder_timer_init()` runs the encoder-side update at `1 kHz`
- The IMU path updates `pitch`, `yaw`, and `roll`
- The encoder path calculates wheel angle, wheel speed, and the motor voltages that actually go to the BLDCs

The control flow is simple in concept:

1. I do not send the target velocity directly to the motors.
2. I first convert velocity error into a `target pitch`.
3. I then compare the current pitch with the target pitch and generate the balancing output.
4. I mix the yaw term into the left and right motor commands as a differential component.
5. I generate `Vq_left` and `Vq_right`, then convert them into 3-phase voltages for the BLDC motors.

In other words, the velocity controller decides how much the body should lean, and the pitch controller keeps the robot from falling while following that lean target.

### 3. Why the velocity estimate is updated every 10 ms

I still read the encoder angle at `1 kHz`, but I do not recompute the wheel velocity from a `1 ms` angle difference every cycle.

At `1 kHz`, the angle change per sample is very small. Because the velocity estimate is `delta angle / delta t`, even a small encoder wobble can look like a very large speed spike when `delta t` is only `1 ms`.

To reduce that effect, `encoder.c` updates the velocity estimate only once every `10` encoder cycles, so the effective velocity-estimation interval becomes `10 ms`.

- the encoder angle is still sampled at `1 kHz`
- only the velocity estimate is decimated to `100 Hz`
- the code unwraps the angle difference at `+/-pi` so one full rotation does not appear as a false jump
- a light `0.5 / 0.5` low-pass filter smooths the remaining noise

In practice, this made the speed feedback much less sensitive to encoder noise without adding a large delay to the balancing controller.

### 4. RTOS scheduling on the real robot

The semaphore-based scheduling in this repository was part of the real hardware code path, not just a simulation-side idea.

- `encoder_timer_init()` configures a GPTimer with `1 MHz` resolution and `alarm_count = 1000`, so the encoder side is triggered every `1 ms` (`1 kHz`).
- Each encoder tick queues SPI reads for both wheel encoders, and `spi_post_callback()` releases `encoder_sem` with `xSemaphoreGiveFromISR()` as soon as the read is complete.
- `motor_control_task` is created at priority `5` and blocks on `xSemaphoreTake(encoder_sem, portMAX_DELAY)`, so the motor-voltage update runs only when fresh encoder data is ready.
- If that encoder semaphore wakes a higher-priority task, `portYIELD_FROM_ISR()` requests an immediate context switch right after the ISR finishes.
- `imu_timer_init()` uses another GPTimer with `IMU_ALARM_COUNT = 5000`, which means the IMU path runs every `5 ms` (`200 Hz`), and its ISR releases `imu_sem` for the state-update path in `app_main()`.
- Other real-time work is also separated: `RX28_Task` runs at priority `4`, and `hc06_event_task` handles Bluetooth packets through the ESP-IDF UART event queue at priority `12`.

This detail mattered a lot on the real robot. Before I used semaphore-based wakeups, the motors vibrated badly because the balancing loop timing was not deterministic. After I made the encoder-triggered motor path higher priority than the IMU path, and woke it directly from the encoder ISR, the vibration disappeared.

### 5. Joystick input

In `hc06.c`, I parse Bluetooth joystick packets in the `S,x,y,diff,E` format.

- Small inputs are ignored with a deadband
- Small joystick magnitude means stop
- Larger forward input becomes a forward velocity target
- Turning is handled as a left/right voltage difference

Because of that structure, forward motion and turning are not handled as separate disconnected modes. The steering command is layered on top of the balancing controller.

### 6. Joint control and expandable hardware

This robot also includes joint actuators, not just wheel control.

- `rx28.c` controls four `RX-28` actuators
- `MAX485` is used for TTL-to-RS485 conversion
- `lidar.c` includes expansion code for `YDLIDAR G2`

The balancing controller is the center of this repository, but the code is organized so I can extend it with more sensing and joint-side behavior later.

---

## Validation process

I did not tune this controller by repeatedly throwing the real robot onto the floor and hoping for the best. I checked it step by step.

### 1. MPPI MATLAB pre-validation

Before relying on the ESP32 MPPI code, I built a MATLAB simulation that mirrors the command-sequence logic in `mppi.c`. The goal was not to simulate every hardware detail. It was to check whether the MPPI controller could generate reasonable `v_ref` and `w_ref` commands when a waypoint target and obstacle field were given.

Simulation code:

- `SoftWare/Mppi_matlab_simulation/main.m`
- `SoftWare/Mppi_matlab_simulation/mppi/`
- `SoftWare/Mppi_matlab_simulation/cost/`
- `SoftWare/Mppi_matlab_simulation/sim/`

MPPI simulation video:

- [MPPI MATLAB simulation test](docs/videos/mppi_test.mp4)

In the MATLAB test, the robot starts from a local coordinate frame, receives a goal point, samples candidate command sequences, and draws both the sampled rollouts and the final weighted MPPI trajectory. The simulation uses world-coordinate obstacle points, but the obstacle cost is reduced into `24` sectors so it stays close to the LiDAR-sector idea used in the ESP32 firmware.

The main simulation parameters are:

- `dt = 0.1 s`
- `horizon = 30`, so the prediction window is `3.0 s`
- `num_samples = 64`
- `v_ref` range: `-0.5 m/s` to `0.5 m/s`
- `w_ref` range: `-1.2 rad/s` to `1.2 rad/s`
- `lambda = 10.0`
- initialized-sequence noise: `v_amp = 0.30`, `w_amp = 0.80`
- first-search noise: `v_amp = 0.40`, `w_amp = 0.90`

The MATLAB cost function is:

```text
total_cost =
  goal_cost
  + obstacle_cost
  + input_cost
  + smooth_cost
```

The weights used in the checked simulation were:

- `weight_goal_x = 1.5`
- `weight_goal_y = 1.5`
- `weight_obstacle = 120.0`
- `weight_input_v = 0.1`
- `weight_input_w = 0.1`
- `weight_smooth_v = 0.3`
- `weight_smooth_w = 0.3`
- `obs_safe_dist = 1.0 m`

This stage gave me a fast way to inspect whether the MPPI command generator was exploring enough candidate trajectories, whether the selected command sequence avoided obstacle clusters, and whether the final trajectory made sense before moving the logic into the ESP32 waypoint code.

### 2. SILS

I first used SILS to see whether the controller response made sense in theory.

<p align="center">
  <img src="docs/images/sils_test.png" alt="SILS test result" width="70%" />
</p>

At this stage, I was mainly checking whether the response diverged, whether the controller could settle, and whether the balancing behavior looked realistic before I touched the real machine.

### 3. HILS

After SILS, I built an HILS setup to verify that the real ESP32 firmware could exchange data correctly with the MATLAB/Simulink model.

![HILS block diagram](docs/images/hils_matlab_block.png)

The point of HILS in this project was not just "simulation." I wanted to confirm that the exact firmware running on the ESP32 followed the same control flow I intended to use on hardware.

To do that, I connected the PC and ESP32 through a `CP2102 USB-to-TTL` module and used `UART2` on the ESP32.

- ESP32 TX: `GPIO17`
- ESP32 RX: `GPIO18`
- PC-ESP32 serial bridge: `CP2102`

#### MATLAB -> ESP32

MATLAB sends four simulated sensor values to the ESP32. In other words, the ESP32 is not receiving arbitrary test numbers here. It is receiving the same kinds of states it would normally read from the real wheel encoders and the IMU.

- from the virtual wheel encoders:
  - right encoder value
  - left encoder value
- from the virtual IMU:
  - pitch
  - yaw

Each value is a `single(float)`, which means 4 bytes per value. That makes one packet `16 bytes` in total.

According to the HILS flow document:

- encoder-side data is refreshed at `1 kHz`
- pitch and yaw are refreshed at `200 Hz`

In practice, MATLAB converts those values into bytes with `Byte Pack`, then sends them over UART so the ESP32 can feed them into the controller.

The data rate also explains why I used a high baud rate:

- `16 bytes * 1000 = 16000 bytes/s`
- UART adds framing overhead because each byte also carries start and stop bits
- the document estimates the required line rate at about `160000 bps`

Because of that, I used `921600 bps` so the HILS link had enough margin.

#### ESP32 -> MATLAB

On the return path, the ESP32 takes the received encoder values and attitude values, runs the controller, and sends back the final 3-phase voltages for the left and right BLDC motors.

That means:

- 3 phases for the left motor
- 3 phases for the right motor
- total `6 float` values

So the return packet size is:

- `6 floats * 4 bytes = 24 bytes`

In short, the HILS loop works like this:

- MATLAB acts like the sensor side and sends a `16-byte` state packet to the ESP32
- the ESP32 calculates the motor commands and sends back a `24-byte` 3-phase voltage packet

On the Simulink side, `Serial Receive` reads the 24-byte payload first, and `Byte Unpack` reconstructs the 6 float values so the virtual BLDC model can use the same motor commands that would be applied on the real robot.

This stage let me verify the real firmware logic without repeatedly crashing the hardware during early tuning.

### 4. Real robot testing

After HILS, I moved to the physical robot and checked disturbance rejection, velocity response, and Bluetooth driving.

<p align="center">
  <img src="docs/images/real_robot.png" alt="real robot" width="45%" />
  <img src="docs/images/sim_robot.png" alt="simulation robot" width="45%" />
</p>

In the real tests, I focused on three things:

- whether the robot could recover after an external push
- whether it leaned naturally to follow a speed target
- whether Bluetooth teleoperation could be added without breaking balance

One of the main hardware-side lessons was scheduling stability. The semaphore and priority structure was not just a clean software design choice. It was what removed the severe motor vibration I saw before the encoder-driven motor path was allowed to preempt the slower IMU path.

---

## Media

GitHub does not always render repository `mp4` files nicely inside `README.md`, so I organized the videos as links.

- [HILS test video](docs/videos/hils_test.mp4)
- [SILS yaw test video](docs/videos/sils_yaw_test.mp4)
- [SILS target velocity test video](docs/videos/sils_target_velocity_test.mp4)
- [SILS disturbance rejection test](docs/videos/sils_disturbance_rejection.mp4)
- [MPPI MATLAB simulation test](docs/videos/mppi_test.mp4)
- [Real-world disturbance rejection test](docs/videos/real_world_disturbance_rejection.mp4)
- [Real-world hold-position test](docs/videos/real_world_hold_position_after_disturbance.mp4)
- [Real-world Bluetooth driving test](docs/videos/real_world_bluetooth_control.mp4)
- [BLDC inverse DQ control video (Q=1)](docs/videos/inverse_dq_control_q1.mp4)
- [BLDC inverse DQ control video (Q=4)](docs/videos/inverse_dq_control_q4.mp4)
- [HILS structure reference PDF](docs/references/hils_flow.pdf)

Additional material:

- [Balancing robot summary PDF](docs/images/balancing_robot_evidence.pdf)

---

## Final hardware stack

These are the main parts I used in the final build:

- Main controller: `ESP32-S3-WROOM-1 N16R8`
- Wheel motors: `GM4108H-120T` BLDC motors x2
- Wheel motor drivers: `MKS SimpleFOC MINI` x2
- Wheel encoders: `AS5048A` x2
- Joint actuators: `Dynamixel RX-28` x4
- Communication converter: `MAX485 TTL to RS-485`
- IMU: `WT901`
- Bluetooth module: `HC-06`
- HILS UART bridge: `CP2102 USB-to-TTL`
- Power: `3S LiPo battery`, `XL6009` boost converters

I also reviewed or used these as support tools or expansion items:

- `U2D2`
- `U2D2 Power Hub`
- `YDLIDAR G2`
- `MPU6050`

For the final balancing tests, the IMU I actually used was `WT901`.

---

## File map

- `SoftWare/Physical_operation_code/`: main ESP-IDF firmware project for the physical balancing robot
- `SoftWare/Physical_operation_code/app_main.c`: entry point, task creation, semaphore waits, and controller integration
- `SoftWare/Physical_operation_code/encoder.c`, `SoftWare/Physical_operation_code/encoder.h`: encoder SPI timing, ISR wakeup, velocity estimation, and 3-phase voltage generation
- `SoftWare/Physical_operation_code/imu.c`, `SoftWare/Physical_operation_code/imu.h`: WT901 data acquisition and the 200 Hz IMU semaphore path
- `SoftWare/Physical_operation_code/pid.c`, `SoftWare/Physical_operation_code/pid.h`: PID calculation
- `SoftWare/Physical_operation_code/pwm.c`, `SoftWare/Physical_operation_code/pwm.h`: PWM output
- `SoftWare/Physical_operation_code/hc06.c`: Bluetooth joystick input
- `SoftWare/Physical_operation_code/rx28.c`, `SoftWare/Physical_operation_code/rx28.h`: RX-28 control
- `SoftWare/Physical_operation_code/lidar.c`, `SoftWare/Physical_operation_code/lidar.h`: LiDAR expansion code
- `SoftWare/Physical_operation_code/variable.h`: shared control variables and constants
- `SoftWare/Gnss_Mppi/`: GNSS waypoint, LiDAR-sector obstacle, and MPPI command-generation firmware variant
- `SoftWare/Mppi_matlab_simulation/`: MATLAB MPPI pre-validation code used before firmware integration
- `SoftWare/hils_test_code/`: HILS-oriented ESP-IDF project variant

---

## Build

This project is written for `ESP-IDF`.

1. Move into `SoftWare/Physical_operation_code`.
2. Open the ESP-IDF environment.
3. Build the project.

```bash
idf.py build
```

4. Flash the board.

```bash
idf.py -p (PORT) flash
```

5. Open the serial monitor if needed.

```bash
idf.py -p (PORT) monitor
```

If you want to work on the HILS variant instead, move into `SoftWare/hils_test_code` and use the same `idf.py` workflow there.

---

## One-line summary

This project is my attempt to build a self-balancing robot that does not just work once in simulation, but is verified step by step through SILS, HILS, and real hardware tests using the same ESP32 control code.
