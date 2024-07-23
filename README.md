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
[Reference](https://github.com/kittinook/ADVANCED-ROBOTICS.git)

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

To create a map, run the following command:

```bash
ros2 launch robot_bridge caver_mapping.launch.py
```

Once RViz has fully loaded, you need to launch teleop to control the robot for creating map.

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

After you have created a map, you can save it using the following command:

```bash
ros2 launch robot_bridge vmegarover_save_map.launch.py
```

Two file will be saved where you ran the command in the folder maps. The `.pgm` file is the map image and the `.yaml` file is the map metadata.

You can change the map name by rename the `.pgm` and `.yaml` file. You also need to foward in to `.yaml` file and change the name to the same name as `.pgm` file

You can change the map name by renaming both the `.pgm` and `.yaml` files. Additionally, you need to open the `.yaml` file and modify the name within it to match the new `.pgm` file name.

Then, copy both files and paste into the `maps` folder located within the `src` folder.

To use a specific map, you'll need to modify the map file name within the Python launch script. 

Running the following command will open the Python launch script in Visual Studio Code:

```bash
code /src/robot_bridge/lauch/robot_bridge.launch.py
```

### 3.4 Navigation (Nav2)

#### 3.4.1 Cost map

#### 3.4.2 Planner

#### 3.4.3 Controller

