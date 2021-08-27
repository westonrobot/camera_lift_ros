# peripheral_camera_lift

## ROS topic commands

## Run server

```
$ rosrun app_lift lift_server /dev/ttyUSB0
```

## Pub speed control
```
$ rostopic pub /lift_speed app_lift/Lift "speed:20"
```

## Pub goal

- type 0 for reset
- type 1 for position
  
```
rostopic pub /LiftActionServer/goal app_li/LiftActionGoal "header:
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
  speed: 30" 

```