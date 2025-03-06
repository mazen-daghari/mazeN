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

![slam](https://github.com/mazen-daghari/mazeN/blob/334f24de1bf3aeb0b9ea591387faadd7f677bd82/slam.gif)

![Robot Moving](https://github.com/mazen-daghari/mazeN/blob/753a22411766561b1b7b400f00673db399df9285/moving_robot.gif)

Installation

1️⃣ Clone the repository:


git clone (https://github.com/mazen-daghari/mazeN.git)
cd 4wd_ros2_sim

2️⃣ Install dependencies:


rosdep install --from-paths src --ignore-src -r -y

3️⃣ Build the workspace:


colcon build --symlink-install


4️⃣ Source the workspace:

source install/setup.bash
Usage
Launch the Simulation


ros2 launch mazeN  gazebo_model.launch.py


for slam 

ros2 launch slam_toolbox online_async_launch.py params_file:=$(pwd)/src/mazeN/config/mapper_params_online_async.yaml use_sim_time:=true

for amcl 

ros2 run nav2_amcl amcl --ros-args -p use_sim_time:=true


Supported Sensors :

RPLiDAR A1/A2/A3 (or other compatible LiDARs)
Intel RealSense D435i (or other depth cameras)
IMU (MPU6050, BNO055, etc.)
Ultrasonic Sensors (HC-SR04)
Future Work
✅ Nav2
✅ Localization using AMCL
✅ Integration with MoveIt for robotic arm support
✅ RViz Visualization improvements
License
📜 MIT License – Open-source and free to use!
feel free to contact me dagmazen@gmail.com
