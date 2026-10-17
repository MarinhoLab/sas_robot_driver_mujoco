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
        auto robot_driver_mujoco = std::make_shared<sas::RobotDriverMujoco>(configuration, shutdown_signaler);

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
