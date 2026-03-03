/**
 * @file sim_robot_types.h
 * @brief Robot data types matching MotionSDK layout, used for MuJoCo sim interface.
 *
 * Joint order (same as Lite3_MotionSDK):
 *   [0] FL_HipX   [1] FL_HipY   [2] FL_Knee
 *   [3] FR_HipX   [4] FR_HipY   [5] FR_Knee
 *   [6] HL_HipX   [7] HL_HipY   [8] HL_Knee
 *   [9] HR_HipX  [10] HR_HipY  [11] HR_Knee
 */

#pragma once
#include <cstdint>
#include <array>

// ── IMU ──────────────────────────────────────────────────────────────────────
struct SimImuData {
    float angle_roll;            // rad
    float angle_pitch;           // rad
    float angle_yaw;             // rad
    float angular_velocity_roll;  // rad/s
    float angular_velocity_pitch; // rad/s
    float angular_velocity_yaw;   // rad/s
    float acc_x;                  // m/s²
    float acc_y;                  // m/s²
    float acc_z;                  // m/s²
};

// ── Per-joint sensor data ────────────────────────────────────────────────────
struct SimJointData {
    float position;    // rad
    float velocity;    // rad/s
    float torque;      // Nm
    float temperature; // °C
};

// ── 12-joint sensor (read from robot/sim) ───────────────────────────────────
struct SimLegData {
    union {
        SimJointData joint_data[12];
        struct {
            SimJointData fl_leg[3]; // FL: HipX, HipY, Knee
            SimJointData fr_leg[3]; // FR: HipX, HipY, Knee
            SimJointData hl_leg[3]; // HL: HipX, HipY, Knee
            SimJointData hr_leg[3]; // HR: HipX, HipY, Knee
        };
    };
};

// ── Full robot state (what you READ) ────────────────────────────────────────
struct SimRobotData {
    uint32_t    tick;
    SimImuData  imu;
    SimLegData  joint_data;
};

// ── Per-joint command ────────────────────────────────────────────────────────
struct SimJointCmd {
    float position; // rad
    float velocity; // rad/s
    float torque;   // Nm (feedforward)
    float kp;
    float kd;
};

// ── 12-joint command (what you SEND) ────────────────────────────────────────
// Control law: T = kp*(pos_goal - pos_real) + kd*(vel_goal - vel_real) + torque
struct SimRobotCmd {
    union {
        SimJointCmd joint_cmd[12];
        struct {
            SimJointCmd fl_leg[3]; // FL: HipX, HipY, Knee
            SimJointCmd fr_leg[3]; // FR: HipX, HipY, Knee
            SimJointCmd hl_leg[3]; // HL: HipX, HipY, Knee
            SimJointCmd hr_leg[3]; // HR: HipX, HipY, Knee
        };
    };
};
