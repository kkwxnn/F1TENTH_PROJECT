# F1TENTH: Autonomous Ackermann Steering Mobile Robot

https://github.com/user-attachments/assets/e7468762-8f40-447a-a497-539dd0bba7ac

### Obstacle Avoidance
https://github.com/user-attachments/assets/739e911e-5323-4bcc-a1b8-9813c5cff750

## Table of Contents
1. [Hardware](#1-hardware)
   - [1.1 Chassis](#11-chassis)
   - [1.2 Laser Sensor](#12-laser-sensor)
   - [1.3 Sensors](#13-sensors)
   - [1.4 Microcontroller](#14-microcontroller)
   - [1.5 Processor](#15-processor)
   - [1.6 Motor](#16-motor)
   - [1.7 Motor Driver](#17-motor-driver)

2. [Electrical & Firmware](#2-electrical--firmware)
   - [2.1 Electrical Components](#21-electrical-components)
   - [2.2 F1TENTH PCB Schematic](#22-f1tenth-pcb-schematic)
   - [2.3 Firmware](#23-firmware)

3. [High-Level](#3-high-level)
   - [3.1 Docker Setup](#31-docker-setup)
     - [3.1.1 Prerequisites](#311-prerequisites)
     - [3.1.2 Services](#312-services)
     - [3.1.3 Usage](#313-usage)
   - [3.2 TF](#32-tf)
     - [3.2.1 Coordinate Frame Transformations](#321-coordinate-frame-transformations)
     - [3.2.2 Visualization](#322-visualization)
   - [3.3 Localization](#33-localization)
     - [3.3.1 Prepare Sensor for Localization](#331-prepare-sensor-for-localization)
     - [3.3.2 Wheel Odometry (Yawrate Odometry)](#332-wheel-odometry-yawrate-odometry)
   - [3.4 Mapping & Localization by Slam Toolbox](#34-mapping--localization-by-slam-toolbox)
     - [3.4.1 Creating Map](#341-creating-map)
     - [3.4.2 Creating Keepout Zone](#342-creating-keepout-zone)
   - [3.5 Navigation (Nav2)](#35-navigation-nav2)
     - [3.5.1 Costmap](#351-costmap)

4. [Usage](#4-usage)
   - [4.1 Run ydlidar_launch.py](#41-run-ydlidar_launchpy)
   - [4.2 Run robot_bridge.launch](#42-run-robot_bridgelaunch)
   - [4.3 Teleoperation](#43-teleoperation)
   - [4.4 Viapoint Generator](#44-viapoint-generator)
     - [4.4.1 Selecting Point from Map](#441-selecting-point-from-map)
       
## 1. Hardware
<div align="center">
  <img src="https://github.com/user-attachments/assets/7b48d97a-7f7b-4ea8-9d7a-1655bd488fa6" alt="f1tenth pic" width="400">
</div>

### 1.1. Chassis
- [TAMIYA 58720 1/10 R/C 4WD TT-02 Type-SRX Chassis Kit](https://www.tamiyausa.com/shop/110-4wd-shaft-drive-road-tt/rc-tt-02-type-srx-chassis-kit/)
  
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
- [TAMIYA 54612 BRUSHLESS MOTOR 02 Sensored 15.5T 2300KV](https://www.tamiya.com/english/products/54612/index.html) (For main drive motor)
- [MG996R Servo Tower Pro](https://www.towerpro.com.tw/product/mg996r/) (For steering motor)
  
### 1.7. Motor Driver
- [TAMIYA 45070 TBLE-04SR BRUSHLESS ELECTRONIC SPEED CONTROLLER](https://www.tamiya.com/english/products/45070/index.html)

 > [!TIP]
 > **High Point Setup**
 > 1. Press and hold down Set button. LED will light up in the order Red → Green → Orange → Red. Release the Set button when the LED is lit up Red, and it will start to flash Red.
 > 2. Apply full throttle and press Set button once. If procedure has been performed correctly, LED will start to double flash Red.
 > 3. Apply full brake and press Set button once. If procedure has been performed correctly, LED will turn off.
 > 
 > [Tamiya Brushless ESC 04SR Manual](https://www.tamiyausa.com/media/files/45070ml-1206-1b53.pdf)

## 2. Electrical & Firmware
### 2.1. Electrical Components
- Step down DC-DC 4.5-30V to 0.8-30V 12A
- HW-683 Module

### 2.2. F1TENTH PCB Schematic
View full PCB schematic in EasyEDA within [this link](https://u.easyeda.com/join?type=project&key=61e3464f45188bac2c8e41a07c19f58e&inviter=db3150f4d5b848c09e05f7428d4a7d9b)

<div align="center">
  <img src="https://github.com/user-attachments/assets/ac95c9e9-dec3-4970-b7cc-894eba6396ad" alt="Schematic_F1TENTH_V4_2024-07-18" width="500">
  <img src="https://github.com/user-attachments/assets/73847e04-d45c-444e-89ba-6c5d62f37a31" alt="PCB_F1TENTH_V4_2024-07-18" width="310">
</div>

### 2.3. Firmware

Process for Setup Low-Level is within [this link](https://github.com/tanakon-apit/F1TENTH_PROJECT.git)

## 3. High-Level 

### 3.1. Docker Setup

Instructions for using the provided `docker-compose.yml` file to set up and run a micro-ROS agent and a ROS 2 desktop environment with VNC access.

#### 3.1.1. Prerequisites
- [Install Docker](https://www.digitalocean.com/community/tutorials/how-to-install-and-use-docker-on-ubuntu-22-04)
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

- Image: `kkwxnn/f1tenth-vnc:humble`
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
**2. Connect Raspberry Pi via SSH**

- Finding IP Address of wlan0 (wifi)
```
ifconfig
```
- Connect Raspberry Pi your computer via SSH
```
ssh -L 6080:localhost:6080 [username]@[inet addr] 
```
For example: ```ssh -L 6080:localhost:6080 pi@192.168.0.152```

**3. Access the ROS 2 Desktop**

Open a web browser and navigate to http://127.0.0.1:6080/ to access the ROS 2 desktop environment via VNC.

**4. Reset the services**
```ctrl+c```


> [!WARNING]
> If you are unable to connect to the ROS 2 desktop via noVNC, follow these steps:
> 
> - Remove the container
> ```
> docker compose down
> ```
> 
> - Start the services again
> ```
> docker compose up 
> ```
> 
> - Reinstall the libraries and dependencies
> ```
> sudo apt update && rosdep update
> rosdep install --from-paths src
> ```

> [!TIP]
> To avoid the need to reinstall libraries and dependencies, you can push your Docker image to Docker Hub. Follow these steps to do.
> 
> Don't forget to **logout of ubuntu in docker** and **stop the docker** before pushing the image.
> ```
> docker stop [container_id_or_name]
> ```
> - Log in to Docker Hub
>
> ```
> docker login
> ```
> - Commit the container
> ```
> docker commit [container_id_or_name] your-username/your-repository:tag
> ```
>
> For example: ```docker commit 1a2b3c4d5e6f your-username/my-app:latest```
> 
> - Tag Your Docker Image
> ```
> docker tag your-image:tag your-username/your-repository:tag
> ```
> For example: ```docker tag my-app:latest your-username/my-app:latest```
>
> - Push the Image to Docker Hub
>
> ```
> docker push your-username/your-repository:tag
> ```
> For example: ```docker push your-username/my-app:latest```

> [!CAUTION]
> In the event that **Docker crashes** and **Docker images and containers disappear**, but **noVNC is still running**, you will need to repeat the `docker compose up` steps and clear the running noVNC instance by following these steps.
>
> - Identify Running noVNC 
> ```
> ps aux | grep localhost
> ```
> - Terminate the noVNC 
> ```
> sudo kill [Process IDs]
> ```
> For example: ```sudo kill 3322```
> - Restart Docker
> ```
> sudo service docker restart
> ```
> or
> ```
> sudo systemctl restart docker
> ```

### 3.2. TF

<div align="center">
  <img src="https://github.com/user-attachments/assets/e0d1709a-a66c-4ba6-8a19-d0308fb0f621" alt="Robot_Description" width="600">
</div>

#### 3.2.1. Coordinate Frame Transformations

#### 3.2.1.1. base_footprint -> **base_link**

- This transform represents the static relationship between the center of the robot base (`base_footprint`) and the floor (`base_link`).

#### 3.2.1.2. base_link -> **imu**

- This transform describes the static relationship between the robot's base (`base_link`) and the IMU (Inertial Measurement Unit) sensor (`imu`).

#### 3.2.1.3. base_link -> **left_wheel**

- This transform defines the revolute relationship between the robot's base (`base_link`) and the left wheel (`left_wheel`).

#### 3.2.1.4. base_link -> **right_wheel**

- This transform defines the revolute relationship between the robot's base (`base_link`) and the right wheel (`right_wheel`).

#### 3.2.1.5. base_link -> **steer**

- This transform defines the static relationship between the robot's base (`base_link`) and the lidar sensor (`steer`).

#### 3.2.1.6. imu -> **laser_frame**

- This transform defines the static relationship between imu (`imu`) and the lidar sensor (`laser_frame`). 

#### 3.2.1.7. base_link -> **optical_odom**

- This transform defines the static relationship between the robot's base (`base_link`) and the Optical Tracking Odometry Sensor (`optical_odom`).

#### 3.2.2. Visualization

#### 3.2.2.1. Launch Robot Description

- Run the following command to launch the robot's description

```
ros2 launch robot_bridge robot_bridge.launch.py 
```
#### 3.2.2.2. View Transformations

- Execute the following command to watch the relationships between the robot's transformations

```
ros2 run tf2_tools view_frames
```

This command opens a graphical interface displaying the relationships between different frames. The provided link shows an example image capturing these transformations:


<div align="center">
  <img src="https://github.com/user-attachments/assets/61efbc9a-44b0-4ba7-a626-1a6dac1811cc" alt="TF_frames" width="900">
</div>


### 3.3. Localization

#### 3.3.1. Prepare sensor for Localization

1) IMU Sensor

    - Calibration sensor

        To calibrate IMU sensor you need to use STM32Cubeide this calibration file is set up for `STM32L432KC` follow this link to low-level github (branch: CalibrateIMU) [[link]](https://github.com/tanakon-apit/F1TENTH_PROJECT/tree/CalibrateIMU)

        Open up the project `BNO_Calibrate` in STM32Cubeide.

        Monitor the `Calibrated` value in the debugger's live expressions to track calibration progress (Accel, Gyro, Magneto).

        - `Magnetometer`: Hold the sensor parallel to the ground and move it in a figure 8 pattern.
        - `Accelerometer`: Place the BNO055 sensor in these six stable positions for a few seconds each.
        - `Gyroscope`: Place the sensor in any stable position for a few seconds.

        After successful calibration, record the BNO offsets from the bno variable in the debugger's live expressions.

        Change branch from `CalibrateIMU` to `Main`, Within the project, navigate to the file `Mobile_Config.h` located in the `Config/inc` directory. Write the BNO offsets from recorded. Then upload code into `STM32L432KC`.

    - Find Offset & Covariance sensor

        - Using `Calibration Node` to find offset and covariance of imu.
            - Collect 10,000 sensor raw data && Save offset and covariance data in YAML file.

            ```
            ros2 run calibration_gen cal_node.py
            ```

            - Copy the information from `f1tenth_ws/install/calibration_gen/share/calibration_gen/config/sensor_calibration.yaml` and paste in `f1tenth_ws/src/calibration_gen/config/sensor_calibration.yaml` 

        - Eliminating noise at stationary state by cutting threshold this will be done by `mcu_bridge` Node.
        
            - `threshold`: 2 x std. x 3.89 (Modeling Noise as Normal Distribution)


    - Verification sensor 

        - Testing Absolute Orientation from imu

            Testing Condition:

            - Turn 90, 180, 360 Degree (R/L)
            - Turn Right then Left 90 Degree


            https://github.com/user-attachments/assets/508171c0-2647-4e11-aa7b-1ab644b267a2


        **The IMU will shift of ±6 degrees each time it completes a full 360-degree rotation.**


2) Laser Sensor 

**Install YDLIDAR ROS2 Driver** [Reference](https://github.com/YDLIDAR/ydlidar_ros2_driver?fbclid=IwZXh0bgNhZW0CMTAAAR3OEcoaB9rG6-haQZFpyFFbUIRuxUHEzv-TmZLJxinNRptzsMPwWTPi2mU_aem_AWq-ZxKVIEgbPc8O5VaWP_GTqjUgGpQF3f1EuqnmXKfztNGkgQNBtLfJfG6miwBMLaj0LysVZxNwI7SLqACmVW_h)

- Clone ydlidar_ros2_driver package for github in `src` folder

```
git clone https://github.com/YDLIDAR/ydlidar_ros2_driver.git 
```
- Clone YDLidar-SDK package for github in `src` folder

```
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
```

- Build & Install [Reference](https://github.com/YDLIDAR/YDLidar-SDK/blob/master/doc/howto/how_to_build_and_install.md)

```
cd YDLidar-SDK
mkdir build
cd build
cmake ..
make
sudo make install
```

- Build ydlidar_ros2_driver package

```
cd [your_workspace]
colcon build --symlink-install
```

#### Usage: Laser Sensor

1. Connect LiDAR
```
ros2 launch ydlidar_ros2_driver ydlidar_launch.py
```

2. Visualization in RVIZ
```
ros2 launch ydlidar_ros2_driver ydlidar_launch_view.py 
```

<div align="center">
  <img src="https://github.com/user-attachments/assets/7b7f3aa2-0fd3-449e-af10-5252123fc776" alt="Lidar" width="550">
</div>

#### 3.3.2. Wheel Odometry (Yawrate Odometry)

**State Equation**

$$ 
\begin{bmatrix}
x_k \\
y_k \\
\theta_k \\
\beta_k \\
v_k
\end{bmatrix}
=\
\begin{bmatrix}
x_{k-1} + v_{k-1} \cdot \Delta t \cdot \cos(\beta_{k-1} + \theta_k) \\
y_{k-1} + v_{k-1} \cdot \Delta t \cdot \sin(\beta_{k-1} + \theta_k) \\
\theta_k \\
0 \\
\text{motor speed} \cdot \text{gear ratio} \cdot r
\end{bmatrix} 
$$

where:
- $x_k$ and $y_k$ are the vehicle's position coordinates at time step $k$.
- $\theta_k$ is the vehicle's orientation (heading angle) at time step $k$.
- $\beta_k$ is the vehicle's slip angle at time step $k$.
- $v_k$ is the vehicle's velocity at time step $k$.

**Verification**

A ceiling-mounted camera in the laboratory serves as a ground truth reference for verifying the yaw rate odometry calculations. These calculations are derived from data obtained from both a hall sensor and an IMU sensor.

<div align="center">
  <img src="https://github.com/user-attachments/assets/81ab683f-14e1-4523-b5d4-ec480f5cf7b3" alt="Camera" width="550">
</div>

#### Experiment 1: Circular Path for 5 Rounds at speed 0.5 m/s

| Position | Odometry Final Position (m) | Camera Final Position (m) | Final Position Error (m) | RMSE (m) |
|----------|-----------------------------|---------------------------|--------------------------|----------|
| X (Y axis of the car frame) | 3.71 | 3.59 | 0.12 | 0.16 |
| Y (X axis of the car frame) | 4.78 | 5.13 | -0.35 | 0.21 |

<div align="center">
  <img src="https://github.com/user-attachments/assets/8b13dcbc-7981-4dc6-95a7-168b0b864c3f" alt="Circular Path" width="550">
</div>

#### Experiment 2: Path Around the Object at speed 0.5 m/s

| Position | Odometry Final Position (m) | Camera Final Position (m) | Final Position Error (m) | RMSE (m) |
|----------|-----------------------------|---------------------------|--------------------------|----------|
| X (Y axis of the car frame) | 2.54 | 2.57 | -0.03 | 0.06 |
| Y (X axis of the car frame) | -1.20 | -1.00 | -0.20 | 0.11 |

<div align="center">
  <img src="https://github.com/user-attachments/assets/464721fc-f9a7-4065-8367-265360b34701" alt="Obj Path" width="550">
</div>

### 3.4. Mapping & Localization by Slam toolbox

#### 3.4.1. Creating Map

To create a map, run the following command:

```bash
ros2 launch robot_bridge caver_mapping.launch.py
```

Once RVIZ has fully loaded, you need to launch teleop to control the robot for creating map.

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
Finally, you will obtain a map similar to the one shown below.

<div align="center">
  <img src="https://github.com/user-attachments/assets/9467d755-7d51-4eaf-bafa-f5f5f1d503a8" alt="Map" width="400">
</div>

#### 3.4.2. Creating Keepout zone

To create a keepout zone, make a copy of both the `.pgm` and `.yaml` files and rename them to `keepout_<map_name>.pgm` and `keepout_<map_name>.yaml` respectively.

Open the `keepout_<map_name>.yaml` file and change the `image` parameter to `keepout_<map_name>.pgm`.

Using image editing software, such as GIMP, draw the area you want to keepout in black. The rest of the image is left as is.

<div align="center">
  <img src="https://github.com/user-attachments/assets/63a5561e-1fa4-4ef0-a66d-c31eb05bc44b" alt="Keepout" width="400">
</div>

### 3.5. Navigation (Nav2)

#### 3.5.1. Costmap

Environment representation used for planning and control. Combines sensor data (depth, AI, semantics) into a grid, assigning costs to cells. Higher costs indicate obstacles or risky areas. [Reference](https://docs.nav2.org/configuration/packages/configuring-costmaps.html)

-  Critical Costmap2D ROS Parameters 
    - `Footprint`

        polygon updated over time due to change of robot's state (base_footprint). If this parameter is set, `isPathValid` will do full collision checking.
        - `Robot_radius`: If footprint coordinates not provided. If this parameter is set, `isPathValid` will do circular collision checking.
    - `plugin`: 
        - `StaticLayer`: 
        
            Static obstacles on the planning space.
            - `footprint_clearing_enabled`: Clear any occupied cells under robot footprint.

        - `ObstacleLayer`:

            Costmap layeruses that use 2D raycasting for 2D lidars, depth, or other sensors. Manages the planning space by the parameters specified
            
            - `obstacle_max_range` / `obstacle_min_range`:
            Maximum, Minimum range to mark obstacles in costmap.
            - `raytrace_max_range` / `raytrace_min_range`
        - `InflationLayer`:

            This layer places an exponential decay functions around obstacles to increase cost to traverse near collision.

            - `inflation_radius`:
            Radius to inflate costmap around lethal obstacles.
            - `cost_scaling_factor`: Exponential decay factor across inflation radius.

-  Costmap2D ROS Parameters tunning guide

    - `Footprint`: For precise navigation in tight spaces, use a detailed robot footprint. In open areas, a simple `Robot_radius` can optimize performance (Collision detection computation time).

    - `obstacle_max_range`-`obstacle_min_range` / `raytrace_max_range`-`raytrace_min_range`: A longer sensor range enhances obstacle detection, improving path planning and collision avoidance. Conversely, a shorter range can expedite computation but limits obstacle awareness.

    - Increasing both `inflation_radius` and `cost_scaling_factor` amplifies the cost area around obstacles. To achieve a stronger cost effect, prioritize increasing `cost_scaling_factor` first (more `cost_scaling_factor` will decrease cost area).

<div align="center">
  <img src="https://github.com/user-attachments/assets/d02a7889-d9a7-4b47-960d-79e825877ede" alt="costmap_lowcost" width="400">
  <img src="https://github.com/user-attachments/assets/1d4ab5ee-5e7b-4775-bc7a-67ccec745f2f" alt="costmap_highcost" width="400">
  
  *`inflation_radius`: low [0.275], high [2.0]*
  
  *`cost_scaling_factor`: low [15.0], high [16.5]*
</div>

 - Example of the nav2 configuration can be found at the following [here](https://github.com/kkwxnn/F1TENTH_PROJECT/blob/humble/f1tenth_ws/src/robot_bridge/config/navigation_param.yaml).

#### 3.5.2. Planner

`Smac Hybrid-A* Planner`

Creates various A* planners for different robots (cars, legged). Supports Hybrid-A* for complex maneuvers (kinematically feasible and support reversing). [Reference](https://docs.nav2.org/configuration/packages/configuring-smac-planner.html)

-  Critical Planner ROS Parameters

    - `allow_unknown`: Allow traversing/search in unknown space.
    - `motion_model_for_search`: Motion model (`Dubin`, `Redds-Shepp`). 
    - `minimum_turning_radius`: Minimum turning radius in meters of vehicle.
    - `reverse_penalty`: Heuristic penalty to apply to SE2 node if searching in reverse direction. Only used in `REEDS_SHEPP` motion model.
    - `non_straight_penalty`: Heuristic penalty to apply to SE2 node if searching in non-straight direction.
    - `cost_penalty`: Heuristic penalty to apply to SE2 node for cost at pose. Allows Hybrid-A* to be cost aware.

 - Example of the nav2 configuration can be found at the following [here](https://github.com/kkwxnn/F1TENTH_PROJECT/blob/humble/f1tenth_ws/src/robot_bridge/config/navigation_param.yaml).

 - Planner ROS Parameters tunning guide: 

    - Reccomend `Dubin` if the robot has not recommend to reverse.

    - Using the exact `minimum_turning_radius` aids robot to follow, while a slightly reduced value can smooth path planning.
    <div align="center">
      <img src="https://github.com/user-attachments/assets/82873879-5055-4781-8e33-f0e27ad4da1a" alt="low_curvature" width="375">
      <img src="https://github.com/user-attachments/assets/9ed54b30-b630-446e-81a9-d49fea4d7f00" alt="high_curvature" width="375">
      
      *`minimum_turning_radius`: low [0.73], high [1.20]*
    </div>

    - Increasing the `non_straight_penalty` heuristic promotes straighter paths in path planning.

    <div align="center">
      <img src="https://github.com/user-attachments/assets/49a3351e-3717-451d-9248-e503268ab6ee" alt="Non_straight_penalty" width="375">
      <img src="https://github.com/user-attachments/assets/1b4493b1-cf73-40d1-98bf-ac7d4272c405" alt="straight_penalty" width="375">
      
      *`non_straight_penalty`: low [1.2], high [2.0]*
    </div>

    - A higher `cost_penalty` incentivizes the planner to avoid high-cost regions, typically obstacles.

#### 3.5.3. Controller

`Regulated Pure Pursuit`

Adaptive speed based on path curvature for safer industrial robot navigation. Prevents overshoot in corners. [Reference](https://docs.nav2.org/configuration/packages/configuring-regulated-pp.html)

If you use the Regulated Pure Pursuit Controller algorithm or software from this repository, please cite this work in your papers:
Macenski, S. Singh, F. Martin, J. Gines, [Regulated Pure Pursuit for Robot Path Tracking](https://arxiv.org/abs/2305.20026). Autonomous Robots, 2023.

- Controller ROS Parameters tunning

    - `desired_linear_vel`: The desired maximum linear velocity (m/s) to use.

    - `use_velocity_scaled_lookahead_dist` is `false`.
        - `lookahead_dist`: The lookahead distance (m) to use to find the lookahead point when 

    - `use_velocity_scaled_lookahead_dist` is `true`.
        - `min_lookahead_dist` / `max_lookahead_dist`: The minimum / maximum lookahead distance (m) threshold when 

        - `lookahead_time`: The time (s) to project the velocity. Also known as the lookahead gain.

    - `use_cost_regulated_linear_velocity_scaling`: Whether to use the regulated features for proximity to obstacles (e.g. slow in close proximity to obstacles).

    - `use_regulated_linear_velocity_scaling`: use the regulated features for path curvature (e.g. slow on high curvature paths).

        - `regulated_linear_scaling_min_radius`: The turning radius (m) for which the regulation features are triggered.

    - `regulated_linear_scaling_min_speed`: To ensure process is still achievable even in high cost spaces with high curvature.

    - `use_fixed_curvature_lookahead`: Use a fixed lookahead distance to compute curvature from.

        - `curvature_lookahead_dist`: Distance to look ahead on the path to detect curvature.

- Controller ROS Parameters tunning guide:

    - `Regulated Pure Pursuit` enhances curve following by adjusting speed through the `regulated_linear_scaling_min_radius` parameter. Higher values reduce speed on tighter curves.

    - Enabling `use_cost_regulated_linear_velocity_scaling` prioritizes obstacle avoidance by reducing speed in high-cost areas.

    - If the area the robot need to traverse high-cost areas, increasing `regulated_linear_scaling_min_speed` enables the robot to traverse by maintaining a minimum velocity.

 - Example of the nav2 configuration can be found at the following [here](https://github.com/kkwxnn/F1TENTH_PROJECT/blob/humble/f1tenth_ws/src/robot_bridge/config/navigation_param.yaml).

 #### 3.5.4. AMCL

 AMCL implements the server for taking a static map and localizing the robot within it using an Adaptive Monte-Carlo Localizer. [Reference](https://docs.nav2.org/configuration/packages/configuring-amcl.html)

 -  Critical AMCL ROS Parameters 

    - Odometry
        - `alpha1`: Expected process noise in odometry’s rotation estimate from rotation.
        - `alpha2`: Expected process noise in odometry’s rotation estimate from translation.
        - `alpha3`: Expected process noise in odometry’s translation estimate from translation.
        - `alpha4`: Expected process noise in odometry’s translation estimate from rotation.
        - `pf_err`: Particle Filter population error.
        - `pf_z`: Particle filter population density. 2.33 is the 99% percentile.
    - Laser (Lidar)
        - `laser_likelihood_max_dist`: Maximum distance to do obstacle inflation on map, for use in likelihood_field model.
        - `sigma_hit`: Standard deviation for Gaussian model used in z_hit part of the model.
        - `z_hit` / `z_rand`: Mixture weight for z_hit part of model, sum of all used z weight must be 1.

- AMCL ROS Parameters tunning guide:

    - Our odometry model is more stable than the laser scan (lidar) data.

    - For a reliable odometry model, reduce expected process noise (`alpha`) and `pf_er`r, while increasing `pf_z`.

    - For unreliable lidar data, increase `z_hit` and `sigma_hit` to incoporate higher measurement noise.

 - Example of the nav2 configuration can be found at the following [here](https://github.com/kkwxnn/F1TENTH_PROJECT/blob/humble/f1tenth_ws/src/robot_bridge/config/navigation_param.yaml).

Parameters prioritize `odometry` over `lidar`.
```
lazer_z_hit = 0.9
laser_sigma_hit = 0.1
laser_z_rand = 0.5
laser_likelihood_max_dist = 4.0
kld_err = 0.01
kld_z = 0.99
odom_alpha1 = 0.005
odom_alpha2 = 0.005
odom_alpha3 = 0.005
odom_alpha4 = 0.005
```
https://github.com/user-attachments/assets/a85da535-ef96-4c38-8924-c9d862924488

Parameters prioritize `lidar` over `odometry`.
```
lazer_z_hit = 0.5
laser_sigma_hit = 0.2
laser_z_rand = 0.5
laser_likelihood_max_dist = 2.0
kld_err = 0.1
kld_z = 0.5
odom_alpha1 = 0.004
odom_alpha2 = 0.04
odom_alpha3 = 0.008
odom_alpha4 = 0.025
```

https://github.com/user-attachments/assets/7fe0a6d9-28dd-49fd-9026-d1f11126844d



 **Verification**

 #### Experiment 1: Path Around the Object at Speed 0.5 m/s

| Position | Camera Last Position (m) | TF Last Position (m) | Error Odom Last Position (m) | Error AMCL Last Position (m) | Error TF Last Position (m) | RMSE TF (m) |
|----------|--------------------------|----------------------|-----------------------------|-----------------------------|--------------------------|-------------|
| X (Y axis of the car frame) | 2.57 | 2.62 | -0.03 | 0.10 | 0.05 | 0.11 |
| Y (X axis of the car frame) | -1.00 | -1.05 | -0.20 | 0.05 | -0.05 | 0.14 |


<div align="center">
  <img src="https://github.com/user-attachments/assets/b8241f82-df3b-4f02-91c4-584b3867b5c1" alt="amcl_tune0.5" width="550">
</div>

#### Experiment 2: Path Around the Object at Speed 2.0 m/s

| Position | Camera Last Position (m) | TF Last Position (m) | Error Odom Last Position (m) | Error AMCL Last Position (m) | Error TF Last Position (m) | RMSE TF (m) |
|----------|--------------------------|----------------------|-----------------------------|-----------------------------|--------------------------|-------------|
| X (Y axis of the car frame) | 2.53 | 2.55 | -0.19 | 0.00 | 0.01 | 0.40 |
| Y (X axis of the car frame) | 0.69 | 0.66 | -0.18 | 0.02 | -0.03 | 0.47 |

<div align="center">
  <img src="https://github.com/user-attachments/assets/d8185f61-b2e4-4700-afda-86f40b62193c" alt="amcl_tune2.0" width="550">
</div>

## 4. Usage

To run the robot, you should first connect to Docker and then run the following terminal commands.

### 4.1. Run ydlidar_launch.py
```
ros2 launch ydlidar_ros2_driver ydlidar_launch.py 
```

### 4.2. Run robot_bridge.launch 
```
ros2 launch robot_bridge robot_bridge.launch.py
```

### 4.3. Teleoperation
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
[Reference](https://index.ros.org/r/teleop_twist_keyboard/)
```
This node takes keypresses from the keyboard and publishes them as Twist
messages. It works best with a US keyboard layout.
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
```
### 4.4. viapoint generator

#### 4.4.1. Selecting point from Map

After lauch robot_bridge.launch.py

```
ros2 topic echo /clicked_point 
```

Click the `Publish Point` button, then select the desired goal point. 

Edit the goal point by modifying the `self.current_goal_pose` value (in [x, y, yaw] format) within the `navigation_param.yaml` file located in the `src/robot_bridge/config` directory.

For example: `self.current_goal_pose` = [[3.6, 0.0, np.deg2rad(0)]]

```
code src/robot_bridge/config/navigation_param.yaml
```

> [!WARNING]
> If your next goal point doesn't generate (Feedback: Reached), consider relocating it or increasing the error threshold. Path planning difficulties might be causing the issue.

After defining all goal points you can execute the Python script from Visual Studio Code or run it as a command below:

```
ros2 run robot_bridge via_points_generator.py
```
