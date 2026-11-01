# Lab 1 – ROS 2, Turtlesim, Actions & Services

Design of Mechatronics Systems – Alexandria National University, Faculty of Engineering

## Team
| Name | ID |
|------|----|
| (mohamed alsayed mohamed) | (2204074) |
| (zed alsayed abdalfadel) | (2204036) |
| (Student 3) | (ID) |

Tested on **ROS 2 Jazzy** (Ubuntu 24.04).

## Packages
| Package | Type | Content |
|---------|------|---------|
| `lab1_interfaces` | ament_cmake | `msg/TargetPose` (x, y, yaw), `action/GoToPose`, `action/DrawSquare`, `srv/SetSpeedLimits` |
| `lab1_turtle` | ament_python | `go_to_pose_server` (action + speed params + speed service), `reset_service`, `square_action_server`, launch files, `config/params.yaml` |

### Nodes, actions and services
| Name | Type | Description |
|------|------|-------------|
| `/go_to_pose` | action `GoToPose` | Goal = `TargetPose` (x, y, yaw). Rotates to the goal, drives, then rotates to the final yaw. Feedback = remaining distance. |
| `/set_speed_limits` | service `SetSpeedLimits` | Changes `max_linear_speed` and `max_angular_speed` live. Refuses values <= 0. |
| `/reset_turtle` | service `std_srvs/Trigger` | Lifts pen → `/clear` → `/turtle1/teleport_absolute` (5.544, 5.544, 0) → pen down. Uses turtlesim services only. |
| `/draw_square` | action `DrawSquare` | Goal = side length. Calls reset, then exactly 4 `go_to_pose` goals. Feedback = current side. |

Parameters of `go_to_pose_server`: `max_linear_speed` (default 2.0), `max_angular_speed` (default 2.0). They are read on **every control cycle**, so changes apply live.

## Build
```bash
sudo apt install ros-jazzy-turtlesim ros-jazzy-rosbag2   # if not installed
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone <THIS_REPO_URL> .
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y
colcon build
source install/setup.bash        # in EVERY new terminal
```

## Run
```bash
ros2 launch lab1_turtle system.launch.py        # turtlesim + all servers + parameters
ros2 launch lab1_turtle square_demo.launch.py   # same, and automatically draws a square after 4 s
```

## Try every requirement
**R2 – interfaces**
```bash
ros2 interface list | grep lab1_interfaces
ros2 interface show lab1_interfaces/msg/TargetPose
ros2 interface show lab1_interfaces/action/GoToPose
ros2 interface show lab1_interfaces/srv/SetSpeedLimits
```
**R3 – goToPose**
```bash
ros2 action list
ros2 action send_goal --feedback /go_to_pose lab1_interfaces/action/GoToPose "{target: {x: 8.0, y: 3.0, yaw: 1.57}}"
```
**R4 – reset**
```bash
ros2 service call /reset_turtle std_srvs/srv/Trigger
ros2 topic echo /turtle1/pose --once      # x≈5.54, y≈5.54, theta≈0
```
**R5 – square**
```bash
ros2 action send_goal --feedback /draw_square lab1_interfaces/action/DrawSquare "{side_length: 3.0}"
```
**R7 – parameters and speed service**
```bash
ros2 param list /go_to_pose_server
ros2 param get  /go_to_pose_server max_linear_speed
ros2 param set  /go_to_pose_server max_linear_speed 0.5
ros2 service call /set_speed_limits lab1_interfaces/srv/SetSpeedLimits "{max_linear_speed: 1.0, max_angular_speed: 1.5}"
ros2 service call /set_speed_limits lab1_interfaces/srv/SetSpeedLimits "{max_linear_speed: -1.0, max_angular_speed: 1.5}"   # refused
```
Try changing the speed while the turtle is moving (send a long goal, then call the service from a second terminal).

## Bonus – Design patterns
* **State** – `lab1_turtle/controller.py`: `PoseController` is a small state machine (`ROTATE_TO_GOAL → DRIVE → ROTATE_TO_YAW → DONE`). Each phase has its own rule, which removes a big `if/else` block from the action server and lets the controller be tested without ROS.
* **Observer** – ROS 2 publish/subscribe: the servers subscribe to `/turtle1/pose` and react to every new pose without polling the turtle.
* **Facade** – `reset_service` hides three turtlesim calls (pen, clear, teleport) behind a single `Trigger` service used by the square server.

## Bonus – Bag file
```bash
# terminal 1
ros2 launch lab1_turtle system.launch.py
# terminal 2: record, then run the square, then Ctrl+C
ros2 bag record -o square_bag /turtle1/cmd_vel /turtle1/pose
ros2 action send_goal --feedback /draw_square lab1_interfaces/action/DrawSquare "{side_length: 3.0}"
ros2 bag info square_bag
# replay: restart turtlesim only (turtle at start), then
ros2 service call /reset_turtle std_srvs/srv/Trigger
ros2 bag play square_bag --topics /turtle1/cmd_vel
```
(Add a screenshot here.)
