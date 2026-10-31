/**
SPDX-License-Identifier: LGPL-3.0-or-later
Copyright (C) 2026 Juan Jose Quiroz Omana

This file is part of sas_robot_driver_mujoco, a SAS robot driver for MuJoCo scenes, based on
cpp-interface-mujoco (https://github.com/juanjqo/cpp-interface-mujoco).
*/

#include <sas_robot_driver_mujoco/sas_robot_driver_mujoco.hpp>
#include <dqrobotics_extensions/interfaces/mujoco/DQ_MujocoInterface.h>
#include <sas_common/sas_object_server.hpp>
#include <sas_common/sas_simulator_server.hpp>
#include <set>
#include <stdexcept>

using DQ_robotics_extensions::DQ_MujocoInterface;

namespace sas
{

/**
 * @brief RobotDriverMujoco::RobotDriverMujoco
 * @param node The node of the topics of the reference frames (that of RobotDriverROS, which spins it).
 * @param configuration The scene, the joints of the robot, their limits, the window, and the reference
 *        frames.
 * @param shutdown_signaler Signals the shutdown (e.g., on SIGINT); the driver also signals it when the
 *        window of the scene is closed.
 */
RobotDriverMujoco::RobotDriverMujoco(const std::shared_ptr<rclcpp::Node>& node,
                                     const RobotDriverMujocoConfiguration& configuration,
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
    if (configuration_.window_width <= 0 || configuration_.window_height <= 0)
        throw std::invalid_argument("RobotDriverMujoco::The window size must be positive.");
    if (configuration_.reference_frames_publish_period < 0)
        throw std::invalid_argument("RobotDriverMujoco::The reference_frames_publish_period cannot be negative.");

    // A sas::ObjectServer for each reference frame: <node name>/<frame>/get/pose and .../set/pose
    std::set<std::string> names;
    for (const auto& frame : configuration_.reference_frames)
    {
        if (frame.name.empty() || !names.insert(frame.name).second)
            throw std::invalid_argument("RobotDriverMujoco::The reference frames need distinct, non-empty names.");
        reference_frame_servers_.push_back(
            std::make_unique<ObjectServer>(node, std::string(node->get_name()) + "/" + frame.name));
    }

    // A sas::SimulatorServer: <node name>/start_simulation and .../stop_simulation pause and resume the
    // simulation, without resetting it (initialize() resets it)
    simulator_server_ = std::make_unique<SimulatorServer>(node, std::string(node->get_name()));
    simulator_server_->set_start_simulation_callback([this]()
    {
        if (mi_)
            mi_->start_simulation();
    });
    simulator_server_->set_stop_simulation_callback([this]()
    {
        if (mi_)
            mi_->stop_simulation();
    });

    // Run by RobotDriverROS at each iteration of its control loop
    set_control_loop_callback([this]()
    {
        if (!mi_)
            return;
        // Closing the window stops the driver, as a SIGINT does
        if (mi_->should_close_window())
            shutdown_signaler_->shutdown();
        _update_reference_frames();
    });
}

RobotDriverMujoco::~RobotDriverMujoco() = default;

/**
 * @brief RobotDriverMujoco::_update_reference_frames moves each reference frame to the last pose
 *        received on its set/pose topic (if any), and publishes their poses on their get/pose topics,
 *        every reference_frames_publish_period.
 */
void RobotDriverMujoco::_update_reference_frames()
{
    for (std::size_t i = 0; i < reference_frame_servers_.size(); i++)
        if (reference_frame_servers_[i]->is_enabled())  // a (unit) pose was received
            mi_->set_reference_frame_pose(configuration_.reference_frames[i].name,
                                          reference_frame_servers_[i]->get_target_pose());

    const auto now = std::chrono::steady_clock::now();
    if (reference_frame_servers_.empty() || now < next_reference_frames_publish_time_)
        return;
    for (std::size_t i = 0; i < reference_frame_servers_.size(); i++)
        reference_frame_servers_[i]->send_pose(mi_->get_reference_frame_pose(configuration_.reference_frames[i].name));
    // From the previous time, so that the period is exact on average (the loop has its own period);
    // after a delay of more than a period, from now
    const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(configuration_.reference_frames_publish_period));
    next_reference_frames_publish_time_ += period;
    if (next_reference_frames_publish_time_ < now)
        next_reference_frames_publish_time_ = now + period;
}

void RobotDriverMujoco::_check_size(const Eigen::VectorXd& v, const std::string& function_name) const
{
    if (v.size() != static_cast<Eigen::Index>(configuration_.robot_joint_names.size()))
        throw std::invalid_argument("RobotDriverMujoco::" + function_name + "::Expected "
                                    + std::to_string(configuration_.robot_joint_names.size())
                                    + " values, got " + std::to_string(v.size()) + ".");
}

/**
 * @brief RobotDriverMujoco::connect loads the scene (and opens its window, unless headless, with the
 *        GUI if gui), and adds the reference frames, at their initial poses.
 * @throws std::runtime_error If gui is set and cpp-interface-mujoco was built without the GUI
 */
void RobotDriverMujoco::connect()
{
    mi_ = std::make_unique<DQ_MujocoInterface>(configuration_.headless ? DQ_MujocoInterface::RENDERING::HEADLESS
                                                                       : DQ_MujocoInterface::RENDERING::WINDOW,
                                               "", configuration_.window_width, configuration_.window_height);
    if (configuration_.gui)
        mi_->set_gui_enabled(true);
    mi_->load_model(configuration_.model_path);
    mi_->get_joint_positions(configuration_.robot_joint_names);  // throws if a joint is not in the scene
    for (const auto& frame : configuration_.reference_frames)
    {
        mi_->add_reference_frame(frame.name, frame.axis_length, frame.axis_width, frame.opacity, frame.show_label);
        mi_->set_reference_frame_pose(frame.name, frame.initial_pose);
    }
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
