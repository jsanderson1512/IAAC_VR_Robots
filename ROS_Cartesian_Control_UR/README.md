# Drafting Deception - ROS Cartesian Control for UR10e

## Installation ##
**Install UR ROS DRIVER [(Beta branch)](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/tree/beta-testing#Building)**

` recommended with ` [Ubuntu 18.04 Realtime Kernel](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/beta-testing/ur_robot_driver/doc/real_time.md) ` and ROS Melodic `

```
source /opt/ros/<your_ros_version>/setup.bash
$ mkdir -p ur_driver/src && cd ur_driver
$ git clone -b beta-testing-boost https://github.com/UniversalRobots/Universal_Robots_Client_Library.git src/Universal_Robots_Client_Library
$ git clone -b beta-testing https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver
$ git clone -b beta-testing https://github.com/fzi-forschungszentrum-informatik/cartesian_ros_control.git src/cartesian_ros_control
$ git clone -b calibration_devel https://github.com/fmauch/universal_robot.git src/fmauch_universal_robot
$ sudo apt update -qq
$ rosdep update
$ rosdep install --from-paths src --ignore-src -y
$ catkin_make_isolated
$ source devel_isolated/setup.bash
```

**Setting up a UR robot for ur_robot_driver**
For using the ur_robot_driver with a real robot you need to install the *externalcontrol-1.0.4.urcap* which can be found inside the [resources folder](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/tree/beta-testing/ur_robot_driver/resources) of this driver.



**Extract calibration information**

```roslaunch ur_calibration calibration_correction.launch robot_ip:=<robot_ip> target_filename:="${HOME}/my_robot_calibration.yaml" ```



## Usage

Copy the files `grasshopper_ur_twist.py` and `cartesian_trajectory_action_client.py` from `IAAC_VR_Robots/ROS_Cartesian_Control_UR/ROS_Scripts/` of this repository, to the `/ur_driver/src/cartesian_ros_control/pass_through_controllers/examples/script/` folder. 

```
$ cd ~/ur_driver
$ catkin_make_isolated
```


### Terminal 1:  UR Bring up

``` 
$ source ~/ur_driver/devel_isolated/setup.bash
$ roslaunch ur_robot_driver <robot_type>_bringup.launch robot_ip:=192.168.56.101 kinematics_config:=$(rospack find ur_calibration)/etc ur10_example_calibration.yaml 
```


### Terminal 2:  [Stopping the scaled joint controller](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/373) and launching pass_through_-_controllers

``` 
$ source ~/ur_driver/devel_isolated/setup.bash
$ rosrun controller_manager controller_manager stop scaled_pos_joint_traj_controller
$ rosrun controller_manager controller_manager spawn pose_based_cartesian_traj_controller
$ roslaunch pass_through_controllers hw_interface.launch
```

### Terminal 3: ROS bridge Server
``` 
$ roslaunch rosbridge_server rosbridge_websocket.launch
```

### Terminal 4: 
```
$ source ~/ur_driver/devel_isolated/setup.bash
$ rosrun pass_through_controllers grasshopper_ur_twist.py
```

*By sending `geoemetry_msgs/Twist` messages on `/target_plane_twist` you can send target planes to the UR.*
