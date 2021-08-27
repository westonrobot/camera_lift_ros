#ifndef APP_LIFT_SERVER_HPP
#define APP_LIFT_SERVER_HPP

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <app_lift/LiftAction.h>
#include <app_lift/LiftGoal.h>
#include <app_lift/Lift.h>

#include "wrp_sdk/peripheral/camera_lift.hpp"

namespace westonrobot {
class LiftActionServer {
 public:
  LiftActionServer(const std::string &port_name)
      : as_(nh_, "LiftActionServer",
            boost::bind(&LiftActionServer::executeCB, this, _1), false),
        action_name_("LiftActionServer"),
        port_name_(port_name) {
    // Action server
    as_.start();

    // Lift setup
    lift_ = std::make_unique<CameraLift>();
    lift_->Connect(port_name);

    // Speed control
    speed_control_subcriber_ = nh_.subscribe<app_lift::Lift>(
        "/lift_speed", 5, &LiftActionServer::SpeedControlCallback, this);
  }

  ~LiftActionServer(void) {}

  void executeCB(const app_lift::LiftGoalConstPtr &goal) {
    bool success = true;

    ROS_INFO(
        "%s: Received goal type of %i with position of %i and speed "
        "of %i",
        action_name_.c_str(), goal->type, goal->position, goal->speed);
    if (lift_->IsOkay()) {
      feedback_.lift_connected = true;

      uint8_t position;
      switch (goal->type) {
        case app_lift::LiftGoal::ResetCmd: {
          lift_->ResetState();
          position = 0;
          break;
        }
        case app_lift::LiftGoal::PositionControlCmd: {
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

        feedback_.speed = lift_->GetLiftState().speed;
        feedback_.position = lift_->GetLiftState().position;

        // publish the feedback
        as_.publishFeedback(feedback_);
      }
      if (success) {
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

  void SpeedControlCallback(const app_lift::Lift::ConstPtr &msg) {
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

 protected:
  ros::NodeHandle nh_;
  ros::Subscriber speed_control_subcriber_;
  actionlib::SimpleActionServer<app_lift::LiftAction>
      as_;  // NodeHandle instance must be created before this line. Otherwise
            // strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  app_lift::LiftFeedback feedback_;
  app_lift::LiftResult result_;

 private:
  std::string port_name_;
  std::unique_ptr<CameraLift> lift_;
  bool speed_control_active_ = false;
};
}  // namespace westonrobot

#endif /* APP_LIFT_SERVER_HPP */
