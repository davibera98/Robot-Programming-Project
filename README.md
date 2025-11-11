# ROBOT PROGRAMMING PROJECT
**Repository for the project of the course Robot Programming**

---

## Overview

ROS node that modulates /cmd_vel using laser scans to prevent collisions during teleoperation.



### Build

1. **Clone the repository**
 ```
git clone https://github.com/davibera98/Robot-Programming-Project
 ```

2. **Enter to the directory**
 ```
cd Robot-Programming-Project
 ```

3. **Build**
 ```
catkin build
 ```

### Run

1. **Start roscore**
Start the roscore in a terminal
  ```
  roscore
  ```

2. **Run the node**
Open a different terminal and run the node
 ```
cd Robot-Programming-Project
source devel/setup.bash
rosrun obstacle_avoidance obstacle_avoidance_node
```
3. **Run teleop**
Open a different terminal and run teleop_twist_keyboard
 ```
cd Robot-Programming-Project
source devel/setup.bash
rosrun teleop_twist_keyboard_cpp teleop_twist_keyboard cmd_vel:=keyboard_vel_call
```
4. **Run the simulator**
Open a different terminal and run the simulator
 ```
cd Robot-Programming-Project
source devel/setup.bash
rosrun stage_ros stageros /home/vboxuser/RP_project/Robot-Programming-Project/src/srrg2_configs/navigation_2d/cappero_laser_odom_diag_obstacle_2020-05-06-16-26-03.world

```

Go to the 3° terminal and move the robot in the simulator using the keyboard


