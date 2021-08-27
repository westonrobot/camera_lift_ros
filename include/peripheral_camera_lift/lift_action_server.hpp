/*
 * lift_action_server.hpp
 *
 * Created on: Aug 27, 2021 15:43
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#ifndef LIFT_ACTION_SERVER_HPP
#define LIFT_ACTION_SERVER_HPP

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <peripheral_camera_lift/LiftAction.h>
#include <peripheral_camera_lift/LiftGoal.h>
#include <peripheral_camera_lift/LiftSpeed.h>
#include <peripheral_camera_lift/LiftState.h>

#include "wrp_sdk/peripheral/camera_lift.hpp"

namespace westonrobot {
class LiftActionServer {
 public:
  LiftActionServer(ros::NodeHandle *nh);
  ~LiftActionServer();

  bool Init(const std::string &port_name, int baud_rate = 115200);
  void PublishLiftState();

 private:
  ros::NodeHandle *nh_;

  // position control action server
  // Note: NodeHandle instance must be created before this line.
  actionlib::SimpleActionServer<peripheral_camera_lift::LiftAction> as_;
  std::string action_name_;

  // create messages that are used to published feedback/result
  peripheral_camera_lift::LiftFeedback feedback_;
  peripheral_camera_lift::LiftResult result_;

  // speed control subscriber
  ros::Subscriber speed_control_subcriber_;

  // state publisher
  ros::Publisher state_publisher_;

  std::string port_name_;
  std::unique_ptr<CameraLift> lift_;
  bool speed_control_active_ = false;

  void Shutdown();

  void ActionExecuteCallback(
      const peripheral_camera_lift::LiftGoalConstPtr &goal);
  void SpeedControlCallback(const peripheral_camera_lift::LiftSpeed::ConstPtr &msg);
};
}  // namespace westonrobot

#endif /* LIFT_ACTION_SERVER_HPP */
