/**
SPDX-License-Identifier: LGPL-3.0-or-later
Copyright (C) 2026 Juan Jose Quiroz Omana

This file is part of sas_robot_driver_mujoco, a SAS robot driver for MuJoCo scenes, based on
cpp-interface-mujoco (https://github.com/juanjqo/cpp-interface-mujoco).
*/

#pragma once
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include <Eigen/Dense>
#include <sas_core/sas_robot_driver.hpp>
#include <sas_core/sas_shutdown_signaler.hpp>

namespace DQ_robotics_extensions
{
class DQ_MujocoInterface;
}

namespace sas
{

/**
 * @brief Configuration of RobotDriverMujoco.
 */
struct RobotDriverMujocoConfiguration
{
    std::string model_path;                                ///< MJCF file of the scene.
    std::vector<std::string> robot_joint_names;            ///< Joints of the robot, in the order of q.
    std::string home_keyframe;                             ///< Keyframe to start from ("": the initial state of the model).
    bool headless = false;                                 ///< Without a window (e.g., in a container or a CI).
    std::tuple<Eigen::VectorXd, Eigen::VectorXd> joint_limits;  ///< (q_min, q_max) in radians.
};

/**
 * @brief RobotDriver for a robot in a MuJoCo scene, simulated in this process with
 *        cpp-interface-mujoco.
 *
 * connect() loads the scene, initialize() resets it (to home_keyframe, if any) and starts the
 * simulation in real time, in the threads of DQ_MujocoInterface, and deinitialize() stops it. The
 * joint commands go to the actuators of the joints, which must be of the kind of the command:
 * position actuators for set_target_joint_positions(), velocity actuators for
 * set_target_joint_velocities(), and motors for set_target_joint_torques().
 *
 * Closing the window of the scene signals the shutdown of the driver (through the control loop
 * callback, which RobotDriverROS runs at each iteration).
 */
class RobotDriverMujoco: public RobotDriver
{
private:
    RobotDriverMujocoConfiguration configuration_;
    std::unique_ptr<DQ_robotics_extensions::DQ_MujocoInterface> mi_;

    void _check_size(const Eigen::VectorXd& v, const std::string& function_name) const;

public:
    RobotDriverMujoco(const RobotDriverMujoco&) = delete;
    RobotDriverMujoco& operator=(const RobotDriverMujoco&) = delete;
    RobotDriverMujoco() = delete;
    ~RobotDriverMujoco();

    RobotDriverMujoco(const RobotDriverMujocoConfiguration& configuration,
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
