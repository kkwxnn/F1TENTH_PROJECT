# F1TENTH: Autonomous Ackermann Steering Mobile Robot

## Table of Contents

## 1. Hardware
### 1.1. Chassis
- TAMIYA 58720 1/10 R/C 4WD TT-02 Type-SRX Chassis Kit
  
### 1.2. Laser Sensor
- [YDLIDAR T-mini Pro](https://www.ydlidar.com/products/view/22.html)

### 1.3. Sensors
- 9-DOF Absolute Orientation IMU Fusion Breakout - BNO055
- [Optical Tracking Odometry Sensor - PAA5160E1](https://www.sparkfun.com/products/24904) (Optional)
  
### 1.4. Microcontroller
- [STM32L432KC](https://www.st.com/en/evaluation-tools/nucleo-l432kc.html)

### 1.5. Processor
- [Raspberry Pi 5 Computer with 8GB RAM](https://www.raspberrypi.com/products/raspberry-pi-5/)
  
### 1.6. Motor
- TAMIYA 54612 BRUSHLESS MOTOR 02 Sensored 15.5T 2300KV (For main drive motor)
- MG996R Servo Tower Pro (For steering motor)
  
### 1.7. Motor Driver
- TAMIYA 45070 TBLE-04SR BRUSHLESS ELECTRONIC SPEED CONTROLLER

## 2. Electrical
### 2.1. Electrical Components
- Step down DC-DC 4.5-30V to 0.8-30V 12A
- HW-683 Module

### 2.2. F1TENTH PCB Schematic
View full PCB schematic in EasyEDA within [this link](https://u.easyeda.com/join?type=project&key=61e3464f45188bac2c8e41a07c19f58e&inviter=db3150f4d5b848c09e05f7428d4a7d9b)

<img src="https://github.com/user-attachments/assets/ac95c9e9-dec3-4970-b7cc-894eba6396ad" alt="Schematic_F1TENTH_V4_2024-07-18" width="500">
<img src="https://github.com/user-attachments/assets/73847e04-d45c-444e-89ba-6c5d62f37a31" alt="PCB_F1TENTH_V4_2024-07-18" width="310">

## 3. High-Level 

### 3.1 Docker Setup

Instructions for using the provided `docker-compose.yml` file to set up and run a micro-ROS agent and a ROS 2 desktop environment with VNC access.

#### 3.1.1. Prerequisites
- Install Docker
- Install Docker Compose

#### 3.1.2. Services
**1. micro_ros_agent** runs a micro-ROS agent that connects to a serial device.

- Image: `microros/micro-ros-agent:humble`
- Container Name: `micro-ros-agent`
- Network Mode: `host`
- Command: `serial --dev /dev/ttyACM0 --baudrate 2000000`
- Devices:
    - `/dev/ttyACM0:/dev/ttyACM0`
- Restart Policy: `unless-stopped`

**2. desktop_ros2** runs a ROS 2 desktop environment with VNC access, allowing you to interact with ROS 2 via a web browser. [Reference](https://github.com/Tiryoh/docker-ros2-desktop-vnc.git)

- Image: `tiryoh/ros2-desktop-vnc:humble`
- Container Name: `desktop_ros2`
- Security Options:
    - `seccomp=unconfined`
- Shared Memory Size: `512m`
- IPC Mode: `host`
- Ports: `6080:80` (Maps port 6080 on your host to port 80 in the container)
- Volumes: 
    - `~/F1TENTH_PROJECT/ros2_ws:/f1tenth_ws:rw` (Mounts your local workspace to the container)
    - `/dev:/dev` (Mounts the host's /dev directory to the container)
- Devices:
    - `/dev/ttyUSB0:/dev/ttyUSB0`
- Restart Policy: `unless-stopped`

#### 3.1.3. Usage
**1. Start the Services**

To start the services defined in the docker-compose.yml file, navigate to the directory containing the file and run:

```
docker compose up 
```

**2. Access the ROS 2 Desktop**

Open a web browser and navigate to http://localhost:6080 to access the ROS 2 desktop environment via VNC.

**3. Reset the services**
```ctrl+c```


#### Warning
If you are unable to connect to the ROS 2 desktop via noVNC, follow these steps:

**Remove the container**
```
docker compose down
```

**Start the services again**
```
docker-compose up 
```

**Reinstall the dependencies**
```
rosdep install --from-paths src
```

### 3.2 Localization

#### 3.2.1 Prepare sensor for Localization

1) IMU Sensor

    - Calibration sensor

    - Find Offset & Covariance sensor

    - Verification sensor 

2) Optical Tracking Odometry Sensor

    - Calibration sensor

    - Verification sensor 

#### 3.2.2 Wheel Odometry (Yawrate Odometry)

- Equation

- Verification

#### 3.2.3 Extended Kalman Filter (EKF)

### 3.3 Mapping & Localization by Slam toolbox

#### 3.3.1. Creating Map

### 3.4 Navigation (Nav2)

#### 3.4.1 Cost map

#### 3.4.2 Planner

#### 3.4.3 Controller

