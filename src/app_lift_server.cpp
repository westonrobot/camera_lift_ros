#include <memory>
#include <csignal>

#include "peripheral_camera_lift/lift_action_server.hpp"

using namespace westonrobot;

bool keep_run = true;

std::unique_ptr<LiftActionServer> server;

int main(int argc, char** argv) {
  ros::init(argc, argv, "LiftActionServer");

  ros::NodeHandle nh;

  std::string port_name;
  nh.param<std::string>("/lift_server/uart_port", port_name, "/dev/ttyUSB0");
  server = std::make_unique<LiftActionServer>(&nh);

  std::cout << "Specified uart port of the lift: " << port_name << std::endl;

  if (!server->Init(port_name)) {
    std::cout << "Failed to initialize lift" << std::endl;
    return -1;
  }

  ros::Rate rate(10);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }

  //   ros::spin();
  return 0;
}