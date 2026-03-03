# Lite3 Custom Gait — Sim Testbed

Write your own gait algorithms and test them in **MuJoCo simulation** before touching the real robot.

The API is deliberately identical to **Lite3_MotionSDK** so porting to hardware is a one-line change.

---

## ✅ Work Accomplished (Final Status)

This testbed has been enhanced with several robust features for Lite3 development:

- **Advanced Keyboard Controller**: Fixed TTY issues using direct `/dev/tty` access. Features WASD movement, speed scaling (`+/-`), and mode toggling.
- **Stair Climbing (Crawl Gait)**: Implemented a research-backed tripod Crawl gait specifically for climbing. It uses one-leg-at-a-time movement with high knee lift (`1.20rad`) for maximum stability.
- **Manual Pitch Control**: Added keys (**F** for lean, **R** for reset) to manually adjust body pitch. This helps keep front feet grounded on steep slopes.
- **State Machine Polish**: Added `Hold()` state for idle stability (no more footing in place) and smooth phase transitions when starting movement.
- **Decoupled Workflow**: Modified Docker setup to allow MuJoCo and the Gait Runner to be restarted independently without a container rebuild.

## ⚠️ Current Status & Limitations
- **Flat Surfaces Only**: The current **Kinematic Gait** (Trot/Crawl) is tuned for stability on **Flat Surfaces only**. 
- **Stairs/Slopes**: While Crawl and Pitch Lean are implemented, they are **not robust for steep slopes or actual stair traversal**. This is the primary reason for moving to the **MPC/WBC** (Dynamic) architecture.

## 📁 Folder Structure

```
Lite3_custom_gait/
├── include/
│   ├── sim_robot_types.h   ← RobotData / RobotCmd structs
│   ├── sim_receiver.h      ← Receives data from MuJoCo (ROS2)
│   ├── sim_sender.h        ← Sends commands to MuJoCo (ROS2)
│   ├── sim_timer.h         ← 1ms real-time timer
│   ├── custom_gait.h       ← Stable Crawl & Trot implementations
│   └── keyboard_controller.h ← Direct TTY keyboard logic
├── src/
│   └── main.cpp            ← Gait State Machine & Logic
├── docker/
│   ├── Dockerfile
│   └── docker-compose.yml  ← Configured for decoupled execution
```

---

## 🚀 How to Run (Decoupled Mode)

1. **Terminal 1 — Reset/Start MuJoCo:**
   ```bash
   docker exec -it lite3_sim_gait /workspace/launch.sh
   ```

2. **Terminal 2 — Start Gait Runner:**
   ```bash
   docker exec -it lite3_sim_gait bash -c "source /workspace/custom_gait_ws/install/setup.bash && ros2 run lite3_custom_gait gait_runner"
   ```

### 🎮 Keyboard Bindings
| Key | Action |
|:---:|---|
| **W / S** | Forward / Backward |
| **A / D** | Turn Left / Right |
| **Q / E** | Strafe Left / Right |
| **T** | **Stair Mode (Crawl)** — Stable 3-leg tripod |
| **F / R** | **Manual Pitch (Lean)** — Keep front feet on ground |
| **+ / -** | Speed Up / Down |
| **Space** | Emergency Stop (Hold Position) |

---

## 📋 MPC/WBC Transition Notice
This repository serves as a **Kinematic Gait Reference**. For advanced Dynamic Control (Centroidal MPC + Whole Body Control), development has moved to the `quadruped-robot` repository which implements QP-based force optimization specifically for the Lite3 robot platform.

---

## 🛡️ Safety Notes
- Press **Ctrl-C** at any time → all joints enter **damping mode** (fixed KD=2.0). 
- The `RobotStateInit()` call applies base damping on startup.
- `JointDamping()` is used as the shutdown hook to ensure the robot collapses safely.
