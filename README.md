# 🚗 Autonomous Driving & Off-Road Path Planning with CARLA

A Python-based autonomous driving simulation project developed using the **CARLA Simulator**, focusing on vehicle control, waypoint-based path following, sensor integration, manual steering, and trajectory analysis.

The project explores autonomous and manual vehicle navigation across both **on-road and off-road environments**, with custom control logic and visualization tools for evaluating vehicle trajectories.

---

##  Key Features

| Feature                         | Description                                                            |
| ------------------------------- | ---------------------------------------------------------------------- |
| 🛣️ **Autonomous Navigation**   | Vehicle navigation using CARLA's route/navigation capabilities         |
| 🌲 **Off-Road Driving**         | Custom waypoint-based path following in off-road environments          |
| 🎯 **Vehicle Control**          | Automatic steering and throttle control                                |
| 📐 **Proportional Steering**    | Steering control based on vehicle position relative to the target path |
| 🎮 **Manual Driving**           | Keyboard and Logitech G29 steering-wheel control                       |
| 📡 **Multi-Sensor Integration** | Camera, LiDAR, Semantic LiDAR, Radar, GNSS and vehicle-event sensors   |
| 🗺️ **Waypoint Visualization**  | Visual representation of generated routes and target points            |
| 📊 **Trajectory Analysis**      | Comparison of manual, automatic and optimal driving trajectories       |
| 🏙️ **Multiple Environments**   | Experiments across CARLA environments including Town7 and Town15       |

---

##  Technical Highlights

### Autonomous Vehicle Control

The project implements automatic vehicle control by continuously evaluating the vehicle's position relative to the target waypoint and calculating steering commands.

A proportional steering approach is used to adjust the vehicle toward the desired path.

```text
              Target Waypoint
                    ●
                   /
                  /
        Vehicle ●
              ↗
       Steering Control
```

The control loop continuously updates the vehicle's steering and throttle commands as it progresses along the route.

---

###  Off-Road Path Following

For off-road scenarios, custom waypoint coordinates are used to define driving paths.

The vehicle follows these waypoints while the controller determines the required steering direction.

```text
Waypoint 1 ───► Waypoint 2
                    \
                     \
                      ► Waypoint 3
                            \
                             ► Waypoint 4
```

This allows autonomous navigation in environments where conventional road-based navigation is not sufficient.

---

###  Multi-Sensor Simulation

The project integrates several CARLA sensors for perception and vehicle-state analysis:

*  RGB Camera
*  LiDAR
*  Semantic LiDAR
*  Radar
*  Collision Sensor
*  Lane-Invasion Sensor
*  GNSS

These sensors provide simulated perception and vehicle information that can be visualized and used during experiments.

---

###  Manual Steering

The project also supports manual vehicle operation.

Driving can be performed using:

* Keyboard controls
* **Logitech G29 steering wheel**

Manual driving trajectories can be recorded and later compared against autonomous and optimal trajectories.

---

##  Trajectory Comparison

One of the analysis components compares three different vehicle trajectories:

```text
                ┌─────────────────────┐
                │   Optimal Route     │
                └──────────┬──────────┘
                           │
              ┌────────────┴────────────┐
              ▼                         ▼
       Automatic Driving          Manual Driving
              │                         │
              └────────────┬────────────┘
                           ▼
                  Trajectory Analysis
                           │
                           ▼
                    Matplotlib Plot
```

The project stores trajectory data and uses **NumPy** and **Matplotlib** to visualize and compare the resulting paths.

This provides a way to evaluate how closely automatic and manual driving follow the desired route.

---

##  Tech Stack

| Technology       | Usage                                                 |
| ---------------- | ----------------------------------------------------- |
| **Python**       | Vehicle control, simulation logic and data processing |
| **CARLA**        | Autonomous-driving simulation                         |
| **NumPy**        | Numerical and trajectory data processing              |
| **Matplotlib**   | Route and trajectory visualization                    |
| **Pygame**       | Vehicle/input interaction                             |
| **Logitech G29** | Manual steering experiments                           |

---

##  Getting Started

### Prerequisites

Install:

* Python 3
* CARLA Simulator
* Required Python packages

Start the CARLA server before running the project.

```bash
./CarlaUE4.sh
```

On Windows:

```bash
CarlaUE4.exe
```

Then run the desired Python simulation script:

```bash
python <script_name>.py
```

> The exact script and CARLA configuration depend on the experiment you want to run.

---

##  Experiments

The project contains experiments covering:

### 1. On-Road Autonomous Driving

Testing autonomous navigation using CARLA's road network and navigation functionality.

### 2. Off-Road Navigation

Following predefined off-road waypoint paths using custom steering control.

### 3. Manual Driving

Controlling the simulated vehicle manually using keyboard input and a Logitech G29 steering wheel.

### 4. Sensor Experiments

Testing and visualizing data from multiple CARLA sensors.

### 5. Route Evaluation

Comparing:

* Optimal trajectory
* Automatically generated trajectory
* Manually driven trajectory

---

##  Skills Demonstrated

This project demonstrates practical experience with:

* **Autonomous vehicle simulation**
* **Python development**
* **Vehicle control algorithms**
* **Path following**
* **Waypoint-based navigation**
* **Sensor integration**
* **Real-time simulation**
* **Data visualization**
* **Trajectory analysis**
* **Debugging and experimentation**
* **Human-in-the-loop vehicle control**

---

##  Project Context

This project was developed as part of work on an **autonomous-driving simulation environment** using CARLA, with experiments involving both autonomous and manual vehicle operation.

The implementation combines simulation, vehicle-control logic, sensor data, visualization, and trajectory analysis to investigate autonomous vehicle behavior in different driving environments.

---
