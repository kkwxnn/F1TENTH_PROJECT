# F1TENTH: Autonomous Ackermann Steering Mobile Robot

This project is a part of FRA532 Mobile Robot, Institute of Field Robotics (FIBO), King's Mongkut's University of Technology Thonburi (KMUTT)

Team Members:
1. Tuchapong Sangthaworn 64340500031
2. Tanakon Apithanakun 64340500062
3. Monsicha Sopitlaptana 64340500071

## Requirements
- Ubuntu 20.04
- ROS2 FOXY

## Table of Contents

- [1. Getting Started](#1-getting-started)
  - [1.1. What you need](#11-what-you-need)
  - [1.2. ROS 2 Setup Environment](#12-ros-2-setup-environment)
  - [1.3. Installation Package](#13-installation-package)
  - [1.4. Connect Jetson Xavier NX via SSH](#14-connect-jetson-xavier-nx-via-ssh)
  - [1.5. Creating the micro-ROS agent](#15-creating-the-micro-ros-agent)
  - [1.6. Running the micro-ROS app](#16-running-the-micro-ros-app)
  - [1.7. Verify Installation: Running Robot (Teleoperation)](#17-verify-installation-running-robot-teleoperation)
- [2. Firmware](#2-firmware)
- [3. TF](#3-tf)
  - [3.1 Sensor and Coordinate Frame Transformations](#31-sensor-and-coordinate-frame-transformations)
  - [3.2 Visualization](#32-visualization)
- [4. Odometry](#4-odometry)
  - [4.1. Odometry Calculation](#41-odometry-calculation)
  - [4.2. Odometry Information](#42-odometry-information)
- [5. Sensors](#5-sensors)
  - [5.1 Laser Sensor](#51-laser-sensor)
- [6. Mapping](#6-mapping)
  - [6.1. Creating Map](#61-creating-map)
- [7. Autonomous Navigation](#7-autonomous-navigation)
  - [7.1. Running the micro-ROS app](#71-running-the-micro-ros-app)
  - [7.2. Run ydlidar_launch.py](#72-run-ydlidar_launchpy)
  - [7.3. Run robot_bridge.launch](#73-run-robot_bridgelaunch)

## 1. Getting Started
### 1.1. What you need
#### 1.1.1. Laser Sensor
- YDLIDAR T-mini Pro 
  
#### 1.1.2. IMUs
- BNO-005
  
#### 1.1.3. Microcontroller
- STM32L432KC

#### 1.1.4. Processor
- Jetson Xavier NX
  
#### 1.1.4. Motor
- TAMIYA 54612 BRUSHLESS MOTOR 02 Sensored 15.5T 2300KV (for main drive motor)
- MG996R Servo Tower Pro (for steering motor)
  
#### 1.1.5. Motor Driver
- TAMIYA 45070 TBLE-04SR BRUSHLESS ELECTRONIC SPEED CONTROLLER

### 1.2. ROS 2 Setup Environment
```
mkdir ~/your_workspace
cd ~/your_workspace
colcon build
source install/setup.bash
```
- To automate workspace setup every time you open a new terminal, add the following line to your ```~/.bashrc``` or ```~/.bash_profile``` file:
```
echo "source ~/your_workspace/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
### 1.3. Installation Package
```
cd [your workspace]/src
git clone https://github.com/kkwxnn/FRA532_Mobile_Robot_Project.git
cd ..
colcon build && source install/setup.bash
```
### 1.4. Connect Jetson Xavier NX via SSH
- Finding IP Address of wlan0 (wifi)
```
ifconfig
```
- Connect Jetson Xavier NX with your computer via SSH (do this every new terminal)
```
ssh [username]@[inet addr] -XC
```
For example: ```ssh jetson@10.7.145.17 -XC```

### 1.5. Creating the micro-ROS agent
[Reference](https://micro.ros.org/docs/tutorials/core/first_application_linux/)

- Source the ROS 2 installation
```
source /opt/ros/$ROS_DISTRO/setup.bash
```
- Create a workspace and download the micro-ROS tools
```
mkdir microros_ws
cd microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
```
- Update dependencies using rosdep
```
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y
```
- Build micro-ROS tools and source them
```
colcon build
source install/local_setup.bash
```

### 1.6. Running the micro-ROS app
```
cd microros_ws
source install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 921600
```
- WARNING! If the serial port is not found, you have to run this command ``` sudo chmod 777 /dev/ttyACM0 ```

### 1.7. Verify Installation: Running Robot (Teleoperation)
```
ros2 launch robot_bridge robot_description.launch.py
```
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
## 2. Firmware
Process for Setup low level is within this [link](https://github.com/tanakon-apit/F1TENTH_PROJECT.git)

## 3. TF

### 3.1 Sensor and Coordinate Frame Transformations

#### 3.1.1 base_footprint -> **base_link**

- This transform represents the static relationship between the center of the robot base (`base_footprint`) and the floor (`base_link`).

#### 3.1.2 base_link -> **imu**

- This transform describes the static relationship between the robot's base (`base_link`) and the IMU (Inertial Measurement Unit) sensor (`imu`).

#### 3.1.3 base_link -> **left_wheel**

- This transform defines the revolute relationship between the robot's base (`base_link`) and the left wheel (`left_wheel`).

#### 3.1.4 base_link -> **right_wheel**

- This transform defines the revolute relationship between the robot's base (`base_link`) and the right wheel (`right_wheel`).

#### 3.1.5 base_link -> **steer**

- This transform defines the static relationship between the robot's base (`base_link`) and the lidar sensor (`steer`).

#### 3.1.6 imu -> **laser_frame**

- This transform defines the static relationship between imu (`imu`) and the lidar sensor (`laser_frame`). 

### 3.2 Visualization

#### 3.2.1 Launch Robot Description:
- Run the following command to launch the robot's description:
```
ros2 launch robot_bridge robot_bridge.launch.py 
```
![image](https://github.com/kkwxnn/FRA532_Mobile_Robot_Project/assets/122891621/dafb6f87-7ae8-40b9-95a7-8e46b88c7712)

#### 3.2.2 View Transformations
- Execute the following command to watch the relationships between the robot's transformations:
```
ros2 run tf2_tools view_frames.py
```
This command opens a graphical interface displaying the relationships between different frames. The provided link shows an example image capturing these transformations:

![image](https://github.com/kkwxnn/FRA532_Mobile_Robot_Project/assets/122891621/114b5b94-3abc-42cd-88ec-68d15511fbc8)


## 4. Odometry
### 4.1. Odometry Calculation
Calculated using a Yaw Rate Algorithm:

![image](https://github.com/kkwxnn/FRA532_Mobile_Robot_Project/assets/122891621/d185040b-cd99-4aca-995b-5e9ed1ae5bc8)

where:
- $x_k$ and $y_k$ are the vehicle's position coordinates at time step $k$.
- $\theta_k$ is the vehicle's orientation (heading angle) at time step $k$.
- $\beta_k$ is the vehicle's slip angle at time step $k$.
- $v_k$ is the vehicle's velocity at time step $k$.
- $\omega_k$ is the vehicle's yaw rate at time step $k$.
- $x_{k-1}$, $y_{k-1}$, $\theta_{k-1}$, $\beta_{k-1}$, $v_{k-1}$, and $\omega_{k-1}$ are the corresponding values at the previous time step $k-1$.
- $\Delta t$ is the time interval between the previous and current time steps.
- ${v}_{RL,k}$ is the estimated velocities of the rear left wheel
- ${v}_{RR,k}$ is the estimated velocities of the rear right wheel
  
This matrix provides a comprehensive update rule for the vehicle's state based on the previous state and the motion inputs.

![image](https://github.com/kkwxnn/FRA532_Mobile_Robot_Project/assets/122891621/0f598d8c-70a4-49eb-9619-f397d7c45a40)

### 4.2. Odometry Information
Do step [1.6](#16-running-the-micro-ros-app) and [1.7](#17-running-robot-teleoperation)
- Get Odometry data
```
ros2 topic echo /odom
```

## 5. Sensors

### 5.1 Laser Sensor

#### 5.1.1 Setup LD06 Lidar

guide provides quick instructions for setting up & testing the LD06 Lidar. For a more comprehensive explanation of LD06 Lidar, 
click [here](https://github.com/YDLIDAR/ydlidar_ros2_driver?fbclid=IwZXh0bgNhZW0CMTAAAR3OEcoaB9rG6-haQZFpyFFbUIRuxUHEzv-TmZLJxinNRptzsMPwWTPi2mU_aem_AWq-ZxKVIEgbPc8O5VaWP_GTqjUgGpQF3f1EuqnmXKfztNGkgQNBtLfJfG6miwBMLaj0LysVZxNwI7SLqACmVW_h)

##### 5.1.1.1 Run the driver

1. Connect LiDAR (Testing YDlidar hardware)
```
ros2 launch ydlidar_ros2_driver ydlidar_launch.py
```
Optional Parameters:
* `port` used to override the autodetect and select a specific port.
* `frame_id` used to override the default `laser` frame_id.

- For example:
```
ros2 launch ldlidar ldlidar.launch.py port:=/dev/ttyACM0
```
2. RVIZ
```
ros2 launch ydlidar_ros2_driver ydlidar_launch_view.py 
```
![441467630_390399290647239_5916944124229973333_n](https://github.com/kkwxnn/FRA532_Mobile_Robot_Project/assets/122891621/ec8c4d3e-f6fe-496b-9903-1e3c9b92a857)

3. echo scan topic
```
ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_client 
```
or:
```
ros2 topic echo /scan
```
## 6. Mapping

Do steps [1.6](#16-running-the-micro-ros-app) and [1.7](#17-running-robot-teleoperation) (close RVIZ)

### 6.1. Creating Map
- To create a map, run the following command
```
ros2 launch carver_navigation carver_mapping.launch.py
```
- Save the map, run the following command
```
ros2 run nav2_map_server map_saver_cli -f [map_name]
```
Two files will be saved where you ran the command. The ```.pgm``` file is the map image and the ```.yaml``` file is the map metadata.

![S__24838172](https://github.com/kkwxnn/FRA532_Mobile_Robot_Project/assets/122891621/55a76116-ac01-4617-adf2-df7f1d06ed57)

*In case the map cannot be saved*
1. If RVIZ is open overlapping, **close RVIZ in step 1.6 and open only the RVIZ used for Save Map**
or
2. Refresh rqt_graph

## 7. Autonomous Navigation

To run Autonomous Navigation on the robot’s computer, open 3 new terminals.

### 7.1. Running the micro-ROS app
```
cd microros_ws
source install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 921600
```
### 7.2. Run ydlidar_launch.py
```
cd [your workspace]
ros2 launch ydlidar_ros2_driver ydlidar_launch.py 
```
### 7.3. Run robot_bridge.launch
```
ros2 launch robot_bridge robot_bridge.launch.py 
```

### In case: Changing Map 

You can select the map file by changing the `map name.yaml`

or

specify the map file in robot_bridge/robot_bridge.launch.py as the default
    
```
ros2 launch robot_bridge robot_bridge.launch.py map:=$HOME/<your workspace>/src/robot_bridge/maps/<map name.yaml>
```
![image](https://github.com/kkwxnn/FRA532_Mobile_Robot_Project/assets/122891621/8bb00942-56e4-47e2-8274-35a4c619dbdf)

![image](https://github.com/kkwxnn/FRA532_Mobile_Robot_Project/assets/122891621/a1f3a369-77e8-4260-be51-cfc2481fc96c)

Press the 2D Pose Estimate button, then drag the arrow onto the map at the location you think the robot is.

![image](https://github.com/kkwxnn/FRA532_Mobile_Robot_Project/assets/122891621/311bacc5-a1ba-4516-967b-14c8fcc0d291)

Press the Start-up button to get the Costmap as the picture below.

![image](https://github.com/kkwxnn/FRA532_Mobile_Robot_Project/assets/122891621/4532d9a6-0489-4a88-a3ae-3e65aa35525a)

