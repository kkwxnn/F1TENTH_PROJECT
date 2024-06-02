# F1TENTH: Autonomous Ackermann Steering Mobile Robot
## Requirement
- Ubuntu 20.04
- ROS2 FOXY

## 1. Getting Started
### 1.1. What you need
#### 1.1.1. Laser Sensor
- YDLIDAR T-mini Pro 
  
#### 1.1.2. IMUs
- BNO-005
  
#### 1.1.3. Microcontroller
- STM32L432KC 
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
To automate workspace setup every time you open a new terminal, add the following line to your ```~/.bashrc``` or ```~/.bash_profile``` file:
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
Finding IP Address of wlan0 (wifi)
```
ifconfig
```
Connect Jetson Xavier NX with your computer via SSH (do this every new terminal)
```
ssh jetson@[inet addr] -XC
```
example: ```ssh jetson@10.7.145.17 -XC```

### 1.5. Creating the micro-ROS agent
[https://micro.ros.org/docs/tutorials/core/first_application_linux/]
Source the ROS 2 installation
```
source /opt/ros/$ROS_DISTRO/setup.bash
```
Create a workspace and download the micro-ROS tools
```
mkdir microros_ws
cd microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
```
Update dependencies using rosdep
```
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y
```
Build micro-ROS tools and source them
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
### 1.7. Running Robot (Teleoperation)
```
ros2 launch robot_bridge robot_description.launch.py
```
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
