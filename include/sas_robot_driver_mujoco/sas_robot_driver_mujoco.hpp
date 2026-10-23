/**
SPDX-License-Identifier: LGPL-3.0-or-later
Copyright (C) 2026 Juan Jose Quiroz Omana

This file is part of sas_robot_driver_mujoco, a SAS robot driver for MuJoCo scenes, based on
cpp-interface-mujoco (https://github.com/juanjqo/cpp-interface-mujoco).
*/

#pragma once
#include <chrono>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include <Eigen/Dense>
#include <dqrobotics/DQ.h>
#include <rclcpp/rclcpp.hpp>
#include <sas_core/sas_robot_driver.hpp>
#include <sas_core/sas_shutdown_signaler.hpp>

namespace DQ_robotics_extensions
{
class DQ_MujocoInterface;
}

namespace sas
{

class ObjectServer;
class SimulatorServer;

/**
 * @brief A reference frame drawn in the scene (no physics), whose pose is exposed by a
 *        sas::ObjectServer: <node name>/<name>/get/pose (published) and <node name>/<name>/set/pose
 *        (subscribed, moves the frame).
 */
struct RobotDriverMujocoReferenceFrame
{
    std::string name;
    DQ_robotics::DQ initial_pose = DQ_robotics::DQ(1);  ///< Pose in the world until a pose is set.
    double axis_length = 0.1;                            ///< Length of the axes, in meters.
    double axis_width = 0.005;                           ///< Width of the axes, in meters.
    double opacity = 1.0;                                ///< From 0 (transparent) to 1 (opaque).
    bool show_label = true;                              ///< Show the name of the frame.
};

/**
 * @brief Configuration of RobotDriverMujoco.
 */
struct RobotDriverMujocoConfiguration
{
    std::string model_path;                                ///< MJCF file of the scene.
    std::vector<std::string> robot_joint_names;            ///< Joints of the robot, in the order of q.
    std::string home_keyframe;                             ///< Keyframe to start from ("": the initial state of the model).
    bool headless = false;                                 ///< Without a window (e.g., in a container or a CI).
    int window_width = 1280;                               ///< Size of the window, in screen coordinates.
    int window_height = 920;
    std::tuple<Eigen::VectorXd, Eigen::VectorXd> joint_limits;  ///< (q_min, q_max) in radians.
    std::vector<RobotDriverMujocoReferenceFrame> reference_frames;
    double reference_frames_publish_period = 0.01;         ///< Period of get/pose of the frames, in seconds.
};

/**
 * @brief RobotDriver for a robot in a MuJoCo scene, simulated in this process with
 *        cpp-interface-mujoco.
 *
 * connect() loads the scene (and adds its reference frames), initialize() resets it (to
 * home_keyframe, if any) and starts the simulation in real time, in the threads of
 * DQ_MujocoInterface, and deinitialize() stops it. The joint commands go to the actuators of the
 * joints, which must be of the kind of the command: position actuators for
 * set_target_joint_positions(), velocity actuators for set_target_joint_velocities(), and motors for
 * set_target_joint_torques().
 *
 * In the control loop callback, which RobotDriverROS runs at each iteration (in the thread that spins
 * the node): closing the window of the scene signals the shutdown of the driver, each reference frame
 * moves to the last pose received on its set/pose topic, and their poses are published on their
 * get/pose topics, every reference_frames_publish_period.
 *
 * A sas::SimulatorServer, at the node's name, pauses and resumes the simulation (without resetting it)
 * for a sas::SimulatorClient.
 */
class RobotDriverMujoco: public RobotDriver
{
private:
    RobotDriverMujocoConfiguration configuration_;
    std::unique_ptr<DQ_robotics_extensions::DQ_MujocoInterface> mi_;

    std::vector<std::unique_ptr<ObjectServer>> reference_frame_servers_;  // one per reference frame
    std::unique_ptr<SimulatorServer> simulator_server_;                   // pauses and resumes the simulation
    std::chrono::steady_clock::time_point next_reference_frames_publish_time_;

    void _check_size(const Eigen::VectorXd& v, const std::string& function_name) const;
    void _update_reference_frames();

public:
    RobotDriverMujoco(const RobotDriverMujoco&) = delete;
    RobotDriverMujoco& operator=(const RobotDriverMujoco&) = delete;
    RobotDriverMujoco() = delete;
    ~RobotDriverMujoco();

    /**
     * @param node The node of the topics of the reference frames (that of RobotDriverROS, which spins it).
     */
    RobotDriverMujoco(const std::shared_ptr<rclcpp::Node>& node,
                      const RobotDriverMujocoConfiguration& configuration,
                      const std::shared_ptr<ShutdownSignaler>& shutdown_signaler);

    Eigen::VectorXd get_joint_positions() override;
    void set_target_joint_positions(const Eigen::VectorXd& desired_joint_positions_rad) override;

    Eigen::VectorXd get_joint_velocities() override;
    void set_target_joint_velocities(const Eigen::VectorXd& desired_joint_velocities) override;

    Eigen::VectorXd get_joint_torques() override;
    void set_target_joint_torques(const Eigen::VectorXd& desired_joint_torques) override;

    void connect() override;
    void disconnect() override;

    void initialize() override;
    void deinitialize() override;
};

}
