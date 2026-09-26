# Friction-Adaptive Model Predictive Control for Autonomous Drift on Ackermann Vehicles

[![Build](https://github.com/Gelminaio/drift-mpc-ackermann/actions/workflows/build.yml/badge.svg)](https://github.com/Gelminaio/drift-mpc-ackermann/actions)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-blue)](https://docs.ros.org/en/jazzy/)

A 1:10 scale autonomous vehicle that drives at the limit of tire adhesion — controlling sideslip (**drift**) on low-friction surfaces via **Nonlinear Model Predictive Control** with **online friction estimation**.

<p align="center">
  <img src="media/vehicle_photos/01_hero.jpg" alt="Vehicle Hero Shot" width="65%"/>
</p>

---

## Abstract

This project investigates aggressive trajectory tracking at the limit of tire adhesion for small-scale autonomous ground vehicles. A Nonlinear MPC controller, formulated on an identified dynamic bicycle model with a nonlinear (Pacejka-type) tire model, regulates trajectory tracking while operating in the saturated-friction regime, enabling controlled oversteer (drift) rather than avoiding it. The road–tire friction coefficient is estimated online from inertial and wheel-odometry measurements, and the MPC model and constraints are adapted accordingly, so the controller maintains stability across surfaces with different and varying grip. The full stack runs distributed over a ROS 2 Jazzy network spanning an ESP32 real-time controller, a Raspberry Pi 4 sensor bridge, and a base-station compute host. Validation is performed both in simulation (sim-to-real pipeline with randomized friction) and on the physical platform across multiple low-friction surfaces.

---

## Vehicle Platform

<table align="center">
  <tr>
    <td width="33%" align="center" valign="top">
      <img src="media/vehicle_photos/02_side_view.jpg" alt="Side view" width="100%"/>
      <br/><em>Side view</em>
      <br/><sub>Three-tier chassis hosting sensors, control electronics, and power distribution.</sub>
    </td>
    <td width="33%" align="center" valign="top">
      <img src="media/vehicle_photos/03_top_view.jpg" alt="Top view" width="80%"/>
      <br/><em>Top view</em>
      <br/><sub>Sensor and compute layout — RPLIDAR A1, BNO085 IMU, ESP32, XL4015 buck converter.</sub>
    </td>
    <td width="33%" align="center" valign="top">
      <img src="media/vehicle_photos/04_electronics_detail.jpg" alt="Electronics detail" width="80%"/>
      <br/><em>Electronics detail</em>
      <br/><sub>ESP32 microcontroller with BNO085 9-DoF IMU over I2C, alongside the RPLIDAR USB adapter and Raspberry Pi 4B host.</sub>
    </td>
  </tr>
</table>

---

## Architecture

![Hardware Schematic](media/hardware_schematic.png)

The system is organized in three computational tiers:

- **ESP32 (real-time layer)** — FreeRTOS firmware running per-wheel velocity PID at 100 Hz, hardware quadrature encoder reading, BNO085 IMU acquisition, Ackermann servo control, and a safety supervisor (ARMED/DISARMED state machine, command-timeout soft-stop, per-wheel stall detection, hardware task watchdog). Exposes a micro-ROS node over USB serial publishing `/joint_states`, `/imu/data_raw` and `/steering_angle` at 50 Hz, subscribing to `/cmd_vel` (inverse Ackermann) and `/arm`.
- **Raspberry Pi 4 (sensor bridge)** — LIDAR and camera drivers, micro-ROS agent, wheel odometry, ROS 2 networking over WiFi.
- **Base station (Ubuntu 24.04 desktop)** — heavy compute: dynamic NMPC solver, online friction estimator, mapping, simulation, RViz visualization.

> 📌 **TODO**: replace the Fritzing schematic with a proper system architecture diagram (ROS 2 nodes + data flow).

---

## Hardware Bill of Materials

| Component | Model | Role | Approx. Cost (EUR) |
|---|---|---|---|
| SBC (sensor bridge) | Raspberry Pi 4B (4GB) | Sensors hub, ROS 2 networking | 100 |
| Power supply (RPi) | USB-C Powerbank | RPi mobile power | 20 |
| MCU (real-time control) | ESP32-WROOM-32 | Motor PID, encoders, IMU, servo | 8 |
| IMU | Bosch BNO085 | 9-DoF with onboard sensor fusion | 36 |
| LIDAR | Slamtec RPLIDAR A1 | 2D 360° laser scan (track boundaries) | 110 |
| Camera | Raspberry Pi Camera Module 3 Wide | RGB perception (CSI) | 38 |
| Motor drivers (×2) | BTS7960 (Half-bridge) | DC motor power stage | 17.5 |
| Step-down converters (×5 pack) | XL4015 | 11.1V → 6V (servo rail) and auxiliary rails | 16 |
| Chassis kit (1:10 RC) | Custom assembly | Mechanical base + 2× geared motors w/ quadrature encoders + LD-1501MG steering servo + 3S LiPo 11.1V 6000 mAh battery | 140 (~100 USD + shipping & customs) |
| Drift tires + low-friction surface | Slick PVC RC tires + acrylic/PVC panel | Controlled, repeatable low-grip testing | ~25 |
| **Total** | | | **~510** |

> 📌 **TODO**: add exact links to each component, photos of the physical assembly.

---

## Software Stack

| Layer | Technology |
|---|---|
| Real-time firmware | C++ / Arduino-ESP32 / FreeRTOS, micro-ROS |
| Middleware | ROS 2 Jazzy Jalisco |
| Perception | OpenCV, `camera_ros`, `rplidar_ros` |
| State estimation | `robot_localization` (EKF) + custom EKF and sideslip estimation (Python/C++) |
| Mapping & baseline tracking | `slam_toolbox`, Pure Pursuit / Stanley |
| Control | Nonlinear MPC via [`acados`](https://docs.acados.org/), dynamic bicycle + Pacejka tire model |
| Friction estimation | Model-based recursive least-squares / scikit-learn |
| Simulation | Gazebo Harmonic / NVIDIA Isaac Sim (adjustable friction, domain randomization) |
| Tooling | Docker, PlatformIO, colcon, GitHub Actions |

---

## Repository Structure

```
drift-mpc-ackermann/
├── firmware/           # ESP32 firmware (PlatformIO)
├── ros2_ws/src/        # ROS 2 packages (perception, control, bringup)
├── simulation/         # Gazebo / Isaac Sim worlds and launch files
├── docker/             # Dockerfile and compose for base station
├── docs/               # Technical documentation (MkDocs)
├── notebooks/          # System identification, data analysis, plots
├── scripts/            # Calibration, test, plotting utilities
├── config/             # Shared YAML configs (EKF, MPC)
├── media/              # Diagrams, photos, demo videos
└── README.md
```

---

## Getting Started

> 📌 **TODO**: expand with calibration and deployment details. Firmware, micro-ROS agent and the full sensor bringup are functional.

### Prerequisites

- Ubuntu 24.04 LTS (base station)
- Docker 24+
- ROS 2 Jazzy Jalisco
- PlatformIO IDE (for firmware development)

### Quick Start

```bash
# Clone
git clone https://github.com/Gelminaio/drift-mpc-ackermann.git
cd drift-mpc-ackermann

# Build base-station Docker environment
docker compose -f docker/docker-compose.yml build

# Flash firmware (ESP32 connected via USB)
cd firmware && pio run -e esp32dev --target upload

# On the vehicle (Raspberry Pi): sensors, odometry and TF
ros2 launch ackermann_bringup robot.launch.py

# On the base station: RViz with model, scan, TF and odometry
ros2 launch ackermann_bringup rviz.launch.py
```

The micro-ROS agent runs as a systemd service on the Pi, see [`setup/pi/`](setup/pi/).

Detailed setup, calibration, and deployment instructions: see [`docs/`](docs/).

---

## Project Roadmap

The project is structured in 12 incremental phases, from physical hardware assembly to academic publication:

- [x] **Phase 1** — Hardware assembly & electrical integration
- [x] **Phase 2** — Infrastructure & repository setup
- [x] **Phase 3** — ESP32 firmware: motor drivers, PID, IMU, micro-ROS
- [x] **Phase 4** — ROS 2 bridge & sensor pipelines
- [x] **Phase 5** — Dynamic modeling & system identification (nonlinear tire model)
- [x] **Phase 6** — State estimation (EKF) & sideslip estimation
- [ ] **Phase 7** — Track, racing line & baseline controller
- [ ] **Phase 8** — Simulation & digital twin (friction randomization)
- [ ] **Phase 9** — Dynamic NMPC baseline at the limit of adhesion (acados)
- [ ] **Phase 10** — Online friction estimation
- [ ] **Phase 11** — Friction-adaptive MPC / autonomous drift control
- [ ] **Phase 12** — Validation, benchmarking, technical report

Track progress via [GitHub Milestones](../../milestones).

---

## Results

### Phase 5 — System identification

<p align="center">
  <img src="media/drift_topview.png" alt="Full left lock, speed steps into a drift, overhead video" width="90%"/>
</p>

Slick PVC tires on tiles. Full left lock, wheel speed stepped up until the rear lets
go (30 s): steady drift at 3.5 rad/s, CG sideslip -20 deg. Ground truth from an
overhead phone video tracking two markers on the car, plus IMU and encoders.

- Dynamic single-track model, spool rear axle (two slipping wheels), saturating tires.
  Fitted on identification runs; 1 s ahead on validation runs: yaw rate NRMSE 11-17%,
  sideslip error 1.4-1.6 deg in grip and 3.1-3.9 deg in the drift.
- Rear friction at full slide 0.276 from the fit, 0.248 from straight launches with wheelspin.
- Steering lag 0.17 s from command to yaw response, against 25-40 ms for the servo alone.
- Every parameter with its uncertainty and source:
  [`vehicle_params.yaml`](ros2_ws/src/ackermann_description/config/vehicle_params.yaml).
  Analysis in [`notebooks/`](notebooks/).

### Phase 6 — State estimation

<p align="center">
  <img src="media/ekf_square.png" alt="Rear axle path and heading on a square: video, odometry, robot_localization" width="90%"/>
  <img src="media/sideslip_drift.png" alt="CG sideslip through two drifts: video against the estimates" width="90%"/>
</p>

- ESP32 clock synced to the Pi: IMU and encoder stamps arrive 7 ms old, steady.
- The accelerometer was a stale copy of itself (60% repeated samples, up to 1.3 s): the
  firmware read one IMU packet per poll. Fixed; its means now match the video within 0.1-0.3 m/s².
- robot_localization with wheel speed and gyro, 2 laps of a 1.6 m square: heading within
  5 deg and position within 0.2 m. Odometry alone ends 227 deg off (spool, tight turns).
- Sideslip: the identified model driven by wheel speed and steering is within 1-2 deg of the
  video in grip, 4-9 deg in a drift. The IMU cannot improve it there: once the tires slide,
  yaw rate and acceleration barely change with sideslip. Lidar velocity comes in Phase 7.
- [`notebooks/state_estimation.ipynb`](notebooks/state_estimation.ipynb).

> 📌 **TODO**: populate with figures, plots, and demo videos as phases complete.

Planned deliverables include:
- Trajectory tracking on low-friction surfaces: dynamic NMPC vs. Pure Pursuit / Stanley / kinematic MPC (which lose control at the limit)
- Online friction estimate vs. ground-truth across surfaces with different grip
- Friction-adaptive vs. fixed MPC at a surface-grip transition (the key experiment)
- Sim-to-real transfer gap analysis under randomized friction

---

## Citation

If you use this work, please cite:

```bibtex
@misc{gelmini2026driftmpc,
    title  = {Friction-Adaptive Model Predictive Control for Autonomous Drift on Ackermann Vehicles},
    author = {Gelmini, Pietro},
    year   = {2026},
    note   = {Work in progress},
    url    = {https://github.com/Gelminaio/drift-mpc-ackermann}
}
```

> 📌 **TODO**: update once preprint / report is published on arXiv.

---

## License

This project is released under the [MIT License](LICENSE).

---

## Contact

**Pietro Gelmini** — [gelmini.pietro@gmail.com](mailto:gelmini.pietro@gmail.com)
LinkedIn: [https://www.linkedin.com/in/pietro-gelmini/](https://www.linkedin.com/in/pietro-gelmini/)

---

*This is an active research/engineering project. Architecture, methodology, and results are subject to change as work progresses.*