/**
 * @file sim_receiver.h
 * @brief Receives joint + IMU data from MuJoCo sim over ROS2 topics.
 *        Drop-in replacement for MotionSDK's Receiver when running in simulation.
 *
 * Usage:
 *   SimReceiver receiver(node);
 *   SimRobotData* data = receiver.GetState();  // always valid reference
 */

#pragma once

#include "sim_robot_types.h"

#include <rclcpp/rclcpp.hpp>
#include <drdds/msg/joints_data.hpp>
#include <drdds/msg/imu_data.hpp>

#include <mutex>
#include <atomic>
#include <cstring>

class SimReceiver {
public:
    explicit SimReceiver(rclcpp::Node::SharedPtr node)
        : node_(node)
    {
        std::memset(&state_, 0, sizeof(state_));

        joint_sub_ = node_->create_subscription<drdds::msg::JointsData>(
            "/JOINTS_DATA", rclcpp::SensorDataQoS(),
            [this](const drdds::msg::JointsData::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                const auto& joints = msg->data.joints_data;
                for (int i = 0; i < 12; ++i) {
                    state_.joint_data.joint_data[i].position    = joints[i].position;
                    state_.joint_data.joint_data[i].velocity    = joints[i].velocity;
                    state_.joint_data.joint_data[i].torque      = joints[i].torque;
                    state_.joint_data.joint_data[i].temperature = joints[i].motion_temp;
                }
                state_.tick++;
                data_ready_ = true;
            });

        imu_sub_ = node_->create_subscription<drdds::msg::ImuData>(
            "/IMU_DATA", rclcpp::SensorDataQoS(),
            [this](const drdds::msg::ImuData::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                state_.imu.angle_roll             = msg->data.roll;
                state_.imu.angle_pitch            = msg->data.pitch;
                state_.imu.angle_yaw              = msg->data.yaw;
                state_.imu.angular_velocity_roll  = msg->data.omega_x;
                state_.imu.angular_velocity_pitch = msg->data.omega_y;
                state_.imu.angular_velocity_yaw   = msg->data.omega_z;
                state_.imu.acc_x                  = msg->data.acc_x;
                state_.imu.acc_y                  = msg->data.acc_y;
                state_.imu.acc_z                  = msg->data.acc_z;
            });

        RCLCPP_INFO(node_->get_logger(), "[SimReceiver] Subscribed to /JOINTS_DATA and /IMU_DATA");
    }

    /** Returns a live snapshot of the robot state (thread-safe copy). */
    SimRobotData GetState() {
        std::lock_guard<std::mutex> lock(mutex_);
        return state_;
    }

    /** Block until first joint message arrives (timeout_ms = 0 → wait forever). */
    bool WaitForFirstData(int timeout_ms = 5000) {
        auto start = std::chrono::steady_clock::now();
        while (!data_ready_) {
            rclcpp::spin_some(node_);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            if (timeout_ms > 0) {
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - start).count();
                if (elapsed > timeout_ms) {
                    RCLCPP_WARN(node_->get_logger(), "[SimReceiver] Timeout waiting for sim data!");
                    return false;
                }
            }
        }
        return true;
    }

    bool IsDataReady() const { return data_ready_.load(); }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<drdds::msg::JointsData>::SharedPtr joint_sub_;
    rclcpp::Subscription<drdds::msg::ImuData>::SharedPtr    imu_sub_;

    std::mutex         mutex_;
    SimRobotData       state_{};
    std::atomic<bool>  data_ready_{false};
};
