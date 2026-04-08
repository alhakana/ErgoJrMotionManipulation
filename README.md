# 🚀 ErgoJr Motion Manipulation — IK Circular Motion Demo

## 📌 Overview

This project demonstrates **inverse kinematics based motion manipulation** of the **ErgoJr robotic arm** — both in **CoppeliaSim simulation** and on the **real Poppy ErgoJr robot**.

The robot performs controlled end-effector motion by:

- generating a circular trajectory in 3D space
- discretizing the circle into target points
- moving the IK target between nearest left/right neighbors
- holding each position for a fixed duration
- using IK to automatically reposition the robot joints so that the end-effector follows the desired trajectory

The goal of this project is to explore:

- inverse kinematics workflows in CoppeliaSim and on real hardware
- motion generation in Cartesian space
- interaction between dynamic joint motors and IK control
- geometric trajectory planning
- transferring simulation behavior to a physical robot

---

## 🎥 Simulation Demo

📹 **[Simulation video](https://drive.google.com/file/d/1-w6EIuPZczw8It_tyWcOajdP_gzVRqbD/view?usp=share_link)**

---

## 🦾 Robot Model

The robotic arm used in this project is:

- **ErgoJr**
- model obtained from the official Poppy documentation
- simulated inside CoppeliaSim EDU
- physical robot controlled via pypot

Main components:

- IK_target
- IK_tip
- automatically generated IK object
- 6 revolute joints (m1 → m6)
- dynamic joint motors controlled through position mode

---

## ⚙️ How It Works

### 1️⃣ Trajectory Generation

The trajectory is defined as:

- a circle in the **world XZ plane**
- centered at the initial position of lamp_visual
- with configurable radius
```
X = cx + r cos(a)
Y = constant
Z = cz + r sin(a)
```
The circle is discretized into target points (7 in simulation, 5 on real robot).

Each point represents a reachable Cartesian target.

---

### 2️⃣ Motion Logic

During simulation:

1. Random start point is selected
2. The robot stays at each point for:
```
holdTime = 3
```
3. The next point is selected randomly:

- nearest left OR
- nearest right

This produces non-deterministic but constrained motion around the circle.

---

### 3️⃣ Inverse Kinematics

**In CoppeliaSim:** The IK Generator automatically creates the IK environment, group, and constraint chain (`Base → Tip → Target`). The solver uses damped least squares with pose constraint, computing joint angles every simulation step.

**On the real robot (`circle_motion_inverse_kinematics.py`):** The script uses the official Poppy ErgoJr URDF and the [ikpy](https://github.com/Phylliade/ikpy) library to solve inverse kinematics. All 6 motors are active in the IK solver — m4 has full range to trace the circle, while m1, m2, m3, m5, m6 are allowed ±15° of adjustment around their reference values (mirroring how the simulation's damped least squares naturally distributes motion). The IK genuinely computes all motor angles from Cartesian target positions using sequential seeding (each solve starts from the previous solution, same as the simulation starting from the current joint configuration).

When the target moves:

- IK solver computes required joint angles
- joint motors move the robot
- end-effector follows the trajectory

---

## 🧠 Key Implementation Details

### Joint Motors

Dynamic motors are enabled to prevent the robot from collapsing under gravity:
```
sim.setJointMode(..., sim.jointmode_force)
sim.setObjectInt32Param(... motor_enabled ...)
sim.setObjectInt32Param(... ctrl_enabled ...)
```
---

### Trajectory Center

The circle center is computed from:
```
center = sim.getObjectPosition(lamp_visual, -1)
```
and slightly shifted along the Y-axis to keep the trajectory reachable.

---

### Motor Direction Handling (Real Robot)

The Poppy ErgoJr has motors with different encoder orientations. Motors m2, m3, m5, m6 are "indirect" (inverted relative to the URDF convention). The script handles this by negating angles for indirect motors when converting between Poppy motor space and IK solver space:
```python
# Poppy -> IK: negate indirect motors
ik_angle = -poppy_angle  # for m2, m3, m5, m6

# IK -> Poppy: negate back
poppy_angle = -ik_angle   # for m2, m3, m5, m6
```

---

## 🧩 Project Structure
```
ErgoJr
├── IK_target
├── IK_tip
├── IK
├── script (motion logic)
└── robot links & joints
```
Main files:

- **`circle_motion_inverse_kinematics.lua.rtf`** — CoppeliaSim Lua script for trajectory generation & target switching
- **`circle_motion_simulation_stage.ttt`** — CoppeliaSim scene file
- **`circle_motion_inverse_kinematics.py`** — Python script for running the circle motion on the real Poppy ErgoJr using IK (ikpy + pypot)

---

## ▶️ Running the Simulation

1. Open `circle_motion_simulation_stage.ttt` in **CoppeliaSim**
2. Ensure:
   - IK object is enabled
   - joints are dynamic
   - motors are enabled
3. Press **Play**

The robot will start moving the end-effector across circular points.

---

## ▶️ Running on the Real Robot

1. Connect to the Poppy ErgoJr (USB or network)
2. Ensure `ikpy`, `numpy`, and `poppy-ergo-jr` are installed
3. Run:
```bash
python circle_motion_inverse_kinematics.py
```
The robot will:
- move to a bent-arm configuration (m3=-90°, m5=-90°)
- compute 5 target positions on a circle via forward kinematics
- solve inverse kinematics for each target
- trace the circle with random CW/CCW steps (same as simulation)
- return to home position and release motors

---

## 🔬 What We Learned

This project explores practical robotics concepts:

- Cartesian vs joint-space control
- IK solver behavior and stability
- reachability constraints
- dynamic vs kinematic simulation
- trajectory discretization
- transferring IK-based motion from simulation to real hardware
- handling motor direction (direct/indirect) in real servo systems
- URDF-based kinematic modeling with ikpy

---

## 👩‍💻 Authors

Project developed collaboratively by:

- Alya Al Hakan
- Anđelija Mladenović

---

## 🏷️ Tech Stack

- CoppeliaSim
- Lua
- simIK
- Python
- ikpy
- pypot
- Poppy ErgoJr URDF
- Inverse Kinematics
- Dynamics simulation
