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

## 2. Electrical & Firmware
### 2.1. Electrical Components
- Step down DC-DC 4.5-30V to 0.8-30V 12A
- HW-683 Module

### 2.2. F1TENTH PCB Schematic
View full PCB schematic in EasyEDA within [this link](https://u.easyeda.com/join?type=project&key=61e3464f45188bac2c8e41a07c19f58e&inviter=db3150f4d5b848c09e05f7428d4a7d9b)

<img src="https://github.com/user-attachments/assets/ac95c9e9-dec3-4970-b7cc-894eba6396ad" alt="Schematic_F1TENTH_V4_2024-07-18" width="500">
<img src="https://github.com/user-attachments/assets/73847e04-d45c-444e-89ba-6c5d62f37a31" alt="PCB_F1TENTH_V4_2024-07-18" width="310">

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
**2. Connect Raspberry Pi via SSH**

- Finding IP Address of wlan0 (wifi)
```
ifconfig
```
- Connect Raspberry Pi your computer via SSH
```
ssh [username]@[inet addr] 
```
For example: ```ssh pi@10.7.145.17```

**2. Access the ROS 2 Desktop**

Open a web browser and navigate to http://127.0.0.1:6080/ to access the ROS 2 desktop environment via VNC.

**3. Reset the services**
```ctrl+c```


> [!WARNING]
> If you are unable to connect to the ROS 2 desktop via noVNC, follow these steps:
> **Remove the container**
> ```
> docker compose down
> ```
> 
> **Start the services again**
> ```
> docker-compose up 
> ```
> 
> **Reinstall the libraries and dependencies**
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
> - Push the Image to Docker Hub
>
> ```
> docker push your-username/your-repository:tag
> ```
> For example: ```docker push kkwxnn/test:tagname```


### 3.2. TF

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
ros2 run tf2_tools view_frames.py
```

This command opens a graphical interface displaying the relationships between different frames. The provided link shows an example image capturing these transformations:

### 3.3. Localization

#### 3.3.1. Prepare sensor for Localization

1) IMU Sensor

    - Calibration sensor

    - Find Offset & Covariance sensor

    - Verification sensor 

2) Optical Tracking Odometry Sensor

    - Calibration sensor

    - Verification sensor 

3) Laser Sensor 

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

#### 3.3.2. Wheel Odometry (Yawrate Odometry)

**Yaw Rate Equation**

$$
\begin{bmatrix}
x_k \\
y_k \\
\theta_k \\
\beta_k \\
v_k
\end{bmatrix}
=
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

#### 3.4.2. Creating Keepout zone

To create a keepout zone, make a copy of both the `.pgm` and `.yaml` files and rename them to `keepout_<map_name>.pgm` and `keepout_<map_name>.yaml` respectively.

Open the `keepout_<map_name>.yaml` file and change the `image` parameter to `keepout_<map_name>.pgm`.

Using image editing software, such as GIMP, draw the area you want to keepout in black. The rest of the image is left as is.

### 3.5. Navigation (Nav2)

#### 3.5.1. Costmap

#### 3.5.2. Planner

#### 3.5.3. Controller

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
