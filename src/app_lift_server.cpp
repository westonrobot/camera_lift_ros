#include <csignal>
#include "app_lift/app_lift_server.hpp"

using namespace westonrobot;

bool keep_run = true;

void CloseLiftActionServer(int signal) { keep_run = false; }

int main(int argc, char** argv) {
  ros::init(argc, argv, "LiftActionServer");

  std::signal(SIGINT, CloseLiftActionServer);

  std::string port_name;
  if (argc == 2) {
    port_name = {argv[1]};
    std::cout << "Specified port: " << port_name << std::endl;
  } else {
    std::cout << "Usage: app_lift_server <interface>" << std::endl
              << "Example 1: ./lift_server /dev/ttyUSB0" << std::endl;
    return -1;
  }

  LiftActionServer LiftActionServer(port_name);

  // ros::Rate rate(10);
  // while (keep_run) {
  //   ros::spinOnce();
  //   rate.sleep();
  // }
  ros::spin();
  return 0;
}