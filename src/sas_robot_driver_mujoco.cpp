/**
SPDX-License-Identifier: LGPL-3.0-or-later
Copyright (C) 2026 Juan Jose Quiroz Omana

This file is part of sas_robot_driver_mujoco, a SAS robot driver for MuJoCo scenes, based on
cpp-interface-mujoco (https://github.com/juanjqo/cpp-interface-mujoco).
*/

#include <sas_robot_driver_mujoco/sas_robot_driver_mujoco.hpp>
#include <dqrobotics_extensions/interfaces/mujoco/DQ_MujocoInterface.h>
#include <stdexcept>

using DQ_robotics_extensions::DQ_MujocoInterface;

namespace sas
{

/**
 * @brief RobotDriverMujoco::RobotDriverMujoco
 * @param configuration The scene, the joints of the robot, and their limits.
 * @param shutdown_signaler Signals the shutdown (e.g., on SIGINT); the driver also signals it when the
 *        window of the scene is closed.
 */
RobotDriverMujoco::RobotDriverMujoco(const RobotDriverMujocoConfiguration& configuration,
                                     const std::shared_ptr<ShutdownSignaler>& shutdown_signaler)
    : RobotDriver(shutdown_signaler), configuration_(configuration)
{
    if (configuration_.model_path.empty())
        throw std::invalid_argument("RobotDriverMujoco::The model_path is empty.");
    if (configuration_.robot_joint_names.empty())
        throw std::invalid_argument("RobotDriverMujoco::The robot_joint_names are empty.");
    const auto& [q_min, q_max] = configuration_.joint_limits;
    const auto n = static_cast<Eigen::Index>(configuration_.robot_joint_names.size());
    if (q_min.size() != n || q_max.size() != n)
        throw std::invalid_argument("RobotDriverMujoco::The joint limits must have one value per joint ("
                                    + std::to_string(n) + ").");
    set_joint_limits(configuration_.joint_limits);

    // Closing the window stops the driver, as a SIGINT does
    set_control_loop_callback([this]()
    {
        if (mi_ && mi_->should_close_window())
            shutdown_signaler_->shutdown();
    });
}

RobotDriverMujoco::~RobotDriverMujoco() = default;

void RobotDriverMujoco::_check_size(const Eigen::VectorXd& v, const std::string& function_name) const
{
    if (v.size() != static_cast<Eigen::Index>(configuration_.robot_joint_names.size()))
        throw std::invalid_argument("RobotDriverMujoco::" + function_name + "::Expected "
                                    + std::to_string(configuration_.robot_joint_names.size())
                                    + " values, got " + std::to_string(v.size()) + ".");
}

/**
 * @brief RobotDriverMujoco::connect loads the scene (and opens its window, unless headless).
 */
void RobotDriverMujoco::connect()
{
    mi_ = std::make_unique<DQ_MujocoInterface>(configuration_.headless ? DQ_MujocoInterface::RENDERING::HEADLESS
                                                                       : DQ_MujocoInterface::RENDERING::WINDOW);
    mi_->load_model(configuration_.model_path);
    mi_->get_joint_positions(configuration_.robot_joint_names);  // throws if a joint is not in the scene
}

/**
 * @brief RobotDriverMujoco::disconnect closes the scene.
 */
void RobotDriverMujoco::disconnect()
{
    mi_.reset();
}

/**
 * @brief RobotDriverMujoco::initialize resets the scene (to the home keyframe, if any) and starts the
 *        simulation, in real time.
 */
void RobotDriverMujoco::initialize()
{
    if (!mi_)
        throw std::runtime_error("RobotDriverMujoco::initialize::Call connect() first.");
    if (configuration_.home_keyframe.empty())
        mi_->reset_simulation();
    else
        mi_->reset_simulation(configuration_.home_keyframe);
    mi_->start_simulation();
}

/**
 * @brief RobotDriverMujoco::deinitialize stops the simulation.
 */
void RobotDriverMujoco::deinitialize()
{
    if (mi_)
        mi_->stop_simulation();
}

Eigen::VectorXd RobotDriverMujoco::get_joint_positions()
{
    return mi_->get_joint_positions(configuration_.robot_joint_names);
}

void RobotDriverMujoco::set_target_joint_positions(const Eigen::VectorXd& desired_joint_positions_rad)
{
    _check_size(desired_joint_positions_rad, "set_target_joint_positions");
    mi_->set_joint_target_positions(configuration_.robot_joint_names, desired_joint_positions_rad);
}

Eigen::VectorXd RobotDriverMujoco::get_joint_velocities()
{
    return mi_->get_joint_velocities(configuration_.robot_joint_names);
}

void RobotDriverMujoco::set_target_joint_velocities(const Eigen::VectorXd& desired_joint_velocities)
{
    _check_size(desired_joint_velocities, "set_target_joint_velocities");
    mi_->set_joint_target_velocities(configuration_.robot_joint_names, desired_joint_velocities);
}

Eigen::VectorXd RobotDriverMujoco::get_joint_torques()
{
    return mi_->get_joint_forces(configuration_.robot_joint_names);
}

void RobotDriverMujoco::set_target_joint_torques(const Eigen::VectorXd& desired_joint_torques)
{
    _check_size(desired_joint_torques, "set_target_joint_torques");
    mi_->set_joint_target_forces(configuration_.robot_joint_names, desired_joint_torques);
}

}
