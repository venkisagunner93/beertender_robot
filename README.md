# beertender_robot 

[![](https://github.com/venkisagunner93/beertender_robot/workflows/CI/badge.svg)](https://github.com/venkisagunner93/beertender_robot/actions)

This repository is for building high-level software for beertender robot.

TO-DO:

- [ ] Create-2 driver integration
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
    - [ ] Data collection pipeline
        - [ ] Logs
        - [ ] Bag files
        - [ ] Metrics

## Package structure

Coming soon

## How to run?

```sh
cd $HOME
git clone https://github.com/venkisagunner93/beertender_robot.git
cd beertender_robot
catkin_make
source devel/setup.bash
roslaunch bringup robot_bringup.launch
```

Output looks like the following:

![rviz](pics/rviz.png)
