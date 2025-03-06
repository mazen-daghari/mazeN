# mazeN
4WD Robot Simulation (ROS2 Foxy) by Mazenkovic

 4-Wheel Drive Mobile Robot Simulation built in ROS2 Foxy, featuring:

✅ LiDAR for mapping and obstacle detection
✅ Depth Camera for vision-based applications
✅ Teleoperation for manual control
✅ Autonomous Navigation (Nav2) for path planning
✅ Gazebo Simulation for realistic physics
Features
🔹 Gazebo Simulation – Fully simulated environment for robot testing
🔹 SLAM & Navigation – Autonomous mapping and path planning with Nav2
🔹 Sensor Suite – Includes LiDAR, IMU, depth camera, ultrasonic sensors
🔹 Teleoperation – Control via keyboard or joystick
🔹 4WD Differential Drive – Supports skid steering and Ackermann steering
🔹 ROS2 Nodes – Modular design with separate nodes for perception, control, and navigation

Installation
1️⃣ Clone the repository:

bash
Copier
Modifier
git clone https://github.com/YOUR_USERNAME/4wd_ros2_sim.git
cd 4wd_ros2_sim
2️⃣ Install dependencies:

bash
Copier
Modifier
rosdep install --from-paths src --ignore-src -r -y
3️⃣ Build the workspace:

bash
Copier
Modifier
colcon build --symlink-install
4️⃣ Source the workspace:

bash
Copier
Modifier
source install/setup.bash
Usage
Launch the Simulation
bash
Copier
Modifier
ros2 launch 4wd_robot_sim bringup.launch.py
Run Teleoperation
bash
Copier
Modifier
ros2 run teleop_twist_keyboard teleop_twist_keyboard
Start Navigation (Nav2)
bash
Copier
Modifier
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=True map:=path/to/map.yaml
Supported Sensors
RPLiDAR A1/A2/A3 (or other compatible LiDARs)
Intel RealSense D435i (or other depth cameras)
IMU (MPU6050, BNO055, etc.)
Ultrasonic Sensors (HC-SR04)
Future Work
✅ Localization using AMCL
✅ Integration with MoveIt for robotic arm support
✅ RViz Visualization improvements
License
📜 MIT License – Open-source and free to use!
