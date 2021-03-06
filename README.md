# Alan

*Alan* is a mobile manipulator composted of a
[Summit XL Steel](https://robotnik.eu/products/mobile-robots/summit-xl-steel-en/) base and
a [Franka Panda](https://www.franka.de/) arm.

This project uses the ROS this two packages for simulation:
- https://github.com/erdalpekel/panda_simulation
- https://github.com/RobotnikAutomation/summit_xl_sim

The installation procedure of all the packages required follows.

## Installation
Before downloading the following ROS packages, it is important that you build 
the **libfranka** library from source and pass its directory to *catkin_make* 
when building the ROS packages as described in [this tutorial](https://frankaemika.github.io/docs/installation.html#building-from-source).

```
mkdir -p catkin_ws/src
cd catkin_ws/src

git clone https://github.com/erdalpekel/panda_simulation.git
git clone https://github.com/erdalpekel/panda_moveit_config.git
git clone --branch simulation https://github.com/erdalpekel/franka_ros.git

git clone https://github.com/RobotnikAutomation/summit_xl_common.git
git clone https://github.com/RobotnikAutomation/robotnik_msgs.git
git clone https://github.com/RobotnikAutomation/robotnik_sensors.git
git clone https://github.com/RobotnikAutomation/summit_xl_sim.git

git clone https://github.com/JenniferBuehler/gazebo-pkgs.git
git clone https://github.com/JenniferBuehler/general-message-pkgs.git

git clone https://github.com/CentroEPiaggio/Alan.git

cd ..
sudo apt-get install libboost-filesystem-dev
rosdep install --from-paths src --ignore-src -y --skip-keys libfranka

catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/opt/ros/kinetic/lib/libfranka/
```

The software has been tested using the following versions of the packages:

```
robotnik_msgs - commit: cdc594d498bcb484c2847d20397bd4d0a294fc33
robotnik_sensors - commit: 681482b384f9d5aa8bca1a355f376b0e9270119b
summit_xl_common - commit: 725f7f127d549a9ed64fa7d829b74628ad0a1040
summit_xl_sim - commit: 9cf27eed144d7be9c3cf21ab529c5046644ccde6
panda_simulation - commit: 9b53b0efc4bbdbf21d6d3c250dd06d495e2aae9f
panda_moveit_config - commit: a145fc96ff906f03c5bf818b32108d0422c2cf3c
franka_ros - commit: 45df2e90fe9e49162397774080284cf6d4c23abd
gazebo-pkgs - commit: baf0f033475c3a592efb0862079f3ff8392cadf6
general-message-pkgs - commit: f0c7a0cc811187cca8e928bc7c5906e463c24945
```

To change the version of a package, run:

```
cd <package folder>
git checkout <commit hash>
```

For example, to change the version of the `summit_xl_common` package run:
```
cd summit_xl_common
git checkout 725f7f127d549a9ed64fa7d829b74628ad0a1040
```

## Running Alan on Gazebo

```
source devel/setup.bash
roslaunch alan sim.launch
```
<img src="docs/images/screenshot.png">
<img src="docs/images/screenshot_rviz.png">

## Example Tasks

### 1 - End Effector position control
Launch Gazebo by running:
```
source devel/setup.bash
roslaunch alan sim.launch
```

In a separate terminal run:
```
roslaunch alan ee_pose_control.launch ee_x:=1.0 ee_y:=1.0 ee_z:=0.7
```

where [`ee_x`, `ee_y`, `ee_z`] is the desired end-effector position.

<img src="docs/images/ee_pose_task.png" >

### 2 - Pick and Place of an object
Launch Gazebo by running:
```
source devel/setup.bash
roslaunch alan sim.launch
```

In a separate terminal run:
```
roslaunch alan move_pick_place.launch
```
The launch file has the following arguments:
- `base_x`: Desired position of the Summit base - X coordinate [default 0.4]
- `base_y`: Desired position of the Summit base - Y coordinate [default 0]
- `base_theta`: Desired yaw of the Summit base [default 0.0]
- `table_x`: Position of the table - X coordinate [default 1.0]
- `table_y`: Position of the table - Y coordinate [default 0.0]
- `table_theta`: Yaw of the table [default 0]
- `object_x`: Position of the object - X coordinate [default 1.0]
- `object_y`: Position of the object - Y coordinate [default 0.0]
- `object_theta`: Yaw of the object [default 0]

<img src="docs/images/pick_place.png" >
