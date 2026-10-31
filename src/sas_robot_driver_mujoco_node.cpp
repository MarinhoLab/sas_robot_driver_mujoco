/**
SPDX-License-Identifier: LGPL-3.0-or-later
Copyright (C) 2026 Juan Jose Quiroz Omana

This file is part of sas_robot_driver_mujoco, a SAS robot driver for MuJoCo scenes, based on
cpp-interface-mujoco (https://github.com/juanjqo/cpp-interface-mujoco).
*/

#include <rclcpp/rclcpp.hpp>
#include <sas_common/sas_common.hpp>
#include <sas_core/eigen3_std_conversions.hpp>
#include <sas_core/sas_shutdown_signaler.hpp>
#include <sas_robot_driver/sas_robot_driver_ros.hpp>
#include <sas_robot_driver_mujoco/sas_robot_driver_mujoco.hpp>
#include <dqrobotics/utils/DQ_Math.h>

/**
 * @brief Reads the reference frames of the parameters: their names, in reference_frame_names, and,
 *        optionally, for each name, reference_frames.<name>.initial_pose ([x, y, z, qw, qx, qy, qz], in
 *        the world), .axis_length, .axis_width, .opacity and .show_label.
 */
static std::vector<sas::RobotDriverMujocoReferenceFrame> get_reference_frames(std::shared_ptr<rclcpp::Node>& node)
{
    std::vector<std::string> names;
    sas::get_ros_optional_parameter(node, "reference_frame_names", names, std::vector<std::string>{});

    std::vector<sas::RobotDriverMujocoReferenceFrame> frames;
    for (const auto& name : names)
    {
        sas::RobotDriverMujocoReferenceFrame frame;
        frame.name = name;
        const std::string prefix = "reference_frames." + name + ".";

        std::vector<double> pose;
        sas::get_ros_optional_parameter(node, prefix + "initial_pose", pose, std::vector<double>{0, 0, 0, 1, 0, 0, 0});
        if (pose.size() != 7)
            throw std::invalid_argument(prefix + "initial_pose must be [x, y, z, qw, qx, qy, qz].");
        const DQ_robotics::DQ r = DQ_robotics::DQ(pose[3], pose[4], pose[5], pose[6]);
        if (r.norm().q(0) < 1e-9)
            throw std::invalid_argument(prefix + "initial_pose has a zero quaternion.");
        const DQ_robotics::DQ r_unit = DQ_robotics::normalize(r);
        frame.initial_pose = r_unit + 0.5*DQ_robotics::E_*(pose[0]*DQ_robotics::i_ + pose[1]*DQ_robotics::j_
                                                           + pose[2]*DQ_robotics::k_)*r_unit;

        sas::get_ros_optional_parameter(node, prefix + "axis_length", frame.axis_length, frame.axis_length);
        sas::get_ros_optional_parameter(node, prefix + "axis_width", frame.axis_width, frame.axis_width);
        sas::get_ros_optional_parameter(node, prefix + "opacity", frame.opacity, frame.opacity);
        sas::get_ros_optional_parameter(node, prefix + "show_label", frame.show_label, frame.show_label);
        frames.push_back(frame);
    }
    return frames;
}

/*********************************************
 * SIGNAL HANDLER
 * *******************************************/
#include <signal.h>
static std::shared_ptr<sas::ShutdownSignaler> shutdown_signaler = std::make_shared<sas::ShutdownSignaler>();
void sig_int_handler(int)
{
    shutdown_signaler->shutdown();
}


int main(int argc, char** argv)
{
    if (signal(SIGINT, sig_int_handler) == SIG_ERR)
        throw std::runtime_error("::Error setting the signal int handler.");

    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    auto node = std::make_shared<rclcpp::Node>("sas_robot_driver_mujoco");

    try
    {
        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "::Loading parameters from parameter server.");

        sas::RobotDriverMujocoConfiguration configuration;
        sas::get_ros_parameter(node, "model_path", configuration.model_path);
        sas::get_ros_parameter(node, "robot_joint_names", configuration.robot_joint_names);
        sas::get_ros_optional_parameter(node, "home_keyframe", configuration.home_keyframe, std::string(""));
        sas::get_ros_optional_parameter(node, "headless", configuration.headless, false);
        sas::get_ros_optional_parameter(node, "gui", configuration.gui, false);
        int64_t window_width, window_height;
        sas::get_ros_optional_parameter(node, "window_width", window_width, int64_t{1280});
        sas::get_ros_optional_parameter(node, "window_height", window_height, int64_t{920});
        configuration.window_width = static_cast<int>(window_width);
        configuration.window_height = static_cast<int>(window_height);
        configuration.reference_frames = get_reference_frames(node);
        sas::get_ros_optional_parameter(node, "reference_frames_publish_period",
                                        configuration.reference_frames_publish_period, 0.01);

        std::vector<double> joint_limits_min;
        std::vector<double> joint_limits_max;
        sas::get_ros_parameter(node, "joint_limits_min", joint_limits_min);  // degrees
        sas::get_ros_parameter(node, "joint_limits_max", joint_limits_max);
        configuration.joint_limits = {DQ_robotics::deg2rad(sas::std_vector_double_to_vectorxd(joint_limits_min)),
                                      DQ_robotics::deg2rad(sas::std_vector_double_to_vectorxd(joint_limits_max))};

        sas::RobotDriverROSConfiguration robot_driver_ros_configuration;
        sas::get_ros_parameter(node, "thread_sampling_time_sec", robot_driver_ros_configuration.thread_sampling_time_sec);
        robot_driver_ros_configuration.robot_driver_provider_prefix = node->get_name();
        robot_driver_ros_configuration.watchdog_period_in_seconds = 0.0;  // the clients enable the watchdog
        robot_driver_ros_configuration.q_min = sas::vectorxd_to_std_vector_double(std::get<0>(configuration.joint_limits));
        robot_driver_ros_configuration.q_max = sas::vectorxd_to_std_vector_double(std::get<1>(configuration.joint_limits));

        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "::Parameters OK.");
        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "::Instantiating RobotDriverMujoco with the scene \""
                                                        + configuration.model_path + "\".");
        auto robot_driver_mujoco = std::make_shared<sas::RobotDriverMujoco>(node, configuration, shutdown_signaler);

        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "::Instantiating RobotDriverROS.");
        sas::RobotDriverROS robot_driver_ros(node,
                                             robot_driver_mujoco,
                                             robot_driver_ros_configuration,
                                             shutdown_signaler);
        robot_driver_ros.control_loop();
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR_STREAM_ONCE(node->get_logger(), std::string("::Exception::") + e.what());
    }

    rclcpp::shutdown();
    return 0;
}
