# beertender_robot 

[![](https://github.com/venkisagunner93/beertender_robot/workflows/CI/badge.svg)](https://github.com/venkisagunner93/beertender_robot/actions)

This repository is for building high-level software for beertender robot.

TO-DO:

- [x] Create-2 driver integration
- [x] Path planning
    - [x] Global planning
    - [x] Local planning
- [ ] Localization
    - [ ] Motion model
    - [ ] Sensor model
- [ ] CI/CD
    - [x] Build pipeline
    - [x] Self-hosted runner
    - [ ] Testing pipeline
        - [ ] gtest/gmock
        - [ ] rostest 
    - [ ] Code quality pipeline
        - [x] clang-format
        - [ ] catkin_lint
        - [ ] roscpp_lint
    - [ ] Deployment pipeline
        - [ ] Containerize deployment
        - [ ] Robot as a docker registry 
    - [ ] Data collection pipeline
        - [ ] Logs
        - [ ] Bag files
        - [ ] Metrics

## How to run?

Prerequisites (install once)
```sh
sudo apt-get update
sudo apt-get install ros-${ROS_DISTRO}-map-server ros-${ROS_DISTRO}-ackermann-msgs ros-${ROS_DISTRO}-joy
```

```sh
cd $HOME
git clone https://github.com/venkisagunner93/beertender_robot.git
cd beertender_robot
catkin_make
source devel/setup.bash
```
For simulation:
```
roslaunch bringup robot_bringup.launch sim:=true
```

Output looks like the following:

![rviz](.images/rviz.png)

## ROS Graph

![ros_graph](.images/ros_graph.png)