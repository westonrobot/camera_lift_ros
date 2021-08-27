# peripheral_camera_lift

## ROS topic commands

## Run server

```
$ roslaunch peripheral_camera_lift camera_lift.launch
```

## Pub speed control
```
$ rostopic pub /lift_speed peripheral_camera_lift/Lift "speed:20"
```

## Pub goal

- type 0 for reset
- type 1 for position
  
```
rostopic pub /LiftActionServer/goal peripheral_camera_lift/LiftActionGoal "header:
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