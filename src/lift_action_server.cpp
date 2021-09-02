/*
 * lift_action_server.cpp
 *
 * Created on: Aug 27, 2021 15:44
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#include "peripheral_camera_lift/lift_action_server.hpp"

namespace westonrobot {
LiftActionServer::LiftActionServer(ros::NodeHandle *nh)
    : nh_(nh),
      as_(*nh_, "LiftActionServer",
          boost::bind(&LiftActionServer::ActionExecuteCallback, this, _1),
          false),
      action_name_("LiftActionServer") {
  speed_control_subcriber_ = nh_->subscribe<peripheral_camera_lift::LiftSpeed>(
      "/lift_speed", 5, &LiftActionServer::SpeedControlCallback, this);
  state_publisher_ = nh_->advertise<peripheral_camera_lift::LiftState>("/lift_state", 10);
}

LiftActionServer::~LiftActionServer() { Shutdown(); }

bool LiftActionServer::Init(const std::string &port_name, int baud_rate) {
  // Lift setup
  lift_ = std::make_unique<CameraLift>();
  lift_->Connect(port_name);
  lift_->Disconnect();
  // Action server
  as_.start();

  return lift_->Connect(port_name);
}

void LiftActionServer::PublishLiftState() { 
  peripheral_camera_lift::LiftState lift_state;
  lift_state.position = lift_->GetLiftState().position;
  lift_state.speed = lift_->GetLiftState().speed;
  state_publisher_.publish(lift_state);
}

void LiftActionServer::Shutdown() {
  as_.shutdown();
  lift_->Disconnect();
}

void LiftActionServer::ActionExecuteCallback(
    const peripheral_camera_lift::LiftGoalConstPtr &goal) {
  bool success = true;

  ROS_INFO(
      "%s: Received goal type of %i with position of %i and speed "
      "of %i",
      action_name_.c_str(), goal->type, goal->position, goal->speed);
  if (lift_->IsOkay()) {
    feedback_.lift_connected = true;

    uint8_t position;
    switch (goal->type) {
      case peripheral_camera_lift::LiftGoal::ResetCmd: {
        lift_->ResetState();
        position = 0;
        break;
      }
      case peripheral_camera_lift::LiftGoal::PositionControlCmd: {
        lift_->SendCommand(goal->position, goal->speed);
        position = goal->position;
        break;
      }
      default:
        break;
    }

    if (position > 100) {
      position = 100;
    } else if (position < 5) {
      position = 0;
    }

    ros::Rate rate(10);
    while ((position != lift_->GetLiftState().position) &&
           !speed_control_active_) {
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok()) {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        lift_->SendCommand(0);
        success = false;
        break;
      }

      // publish the feedback
      feedback_.lift_connected = true;
      feedback_.speed = lift_->GetLiftState().speed;
      feedback_.position = lift_->GetLiftState().position;
      as_.publishFeedback(feedback_);
      rate.sleep();
    }

    if (success) {
      feedback_.lift_connected = true;
      feedback_.speed = lift_->GetLiftState().speed;
      feedback_.position = lift_->GetLiftState().position;
      as_.publishFeedback(feedback_);

      result_.reached = true;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
    speed_control_active_ = false;
  } else {
    feedback_.lift_connected = false;
    as_.publishFeedback(feedback_);
    result_.reached = false;
    as_.setAborted(result_);
  }
}

void LiftActionServer::SpeedControlCallback(
    const peripheral_camera_lift::LiftSpeed::ConstPtr &msg) {
  ROS_INFO("Received speed of %i", msg->speed);
  if (lift_->IsOkay()) {
    // set the action state to preempted
    if (as_.isActive()) {
      ROS_INFO("%s: Preempted by speed control", action_name_.c_str());
      speed_control_active_ = true;
    }
    lift_->SendCommand(msg->speed);
  }
}
}  // namespace westonrobot
