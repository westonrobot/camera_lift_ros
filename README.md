# Peripheral: Camera Lift ROS Support Package

## Install dependencies

* wrp_sdk: >= 0.8.14

```
$ echo "deb https://westonrobot.jfrog.io/artifactory/wrtoolbox-release bionic next" | sudo tee /etc/apt/sources.list.d/weston-robot.list
$ curl -sSL 'https://westonrobot.jfrog.io/artifactory/api/security/keypair/wr-deb/public' | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install wrp_sdk
```

## Build ROS package

```
$ cd <your-catkin-workspace>/src
$ git clone https://github.com/westonrobot/camera_lift_ros.git
$ cd .. 
$ catkin_make
```

## Use the ROS package

* Launch the action server

```
$ roslaunch peripheral_camera_lift camera_lift.launch
```

* Send goal to the action server from command line

- type 0 for reset
- type 1 for position
  
```
$ rostopic pub /LiftActionServer/goal peripheral_camera_lift/LiftActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  type: 1
  position: 100
  speed: 50" 
```

* Publish speed control from command line
```
$ rostopic pub /lift_speed peripheral_camera_lift/Lift "speed: 50"
```

**Note** The lift won't move if the set speed is too low. The payload may affect the minimum speed.
