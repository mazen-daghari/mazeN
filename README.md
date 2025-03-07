# mazeN
4WD Robot Simulation (ROS2 Foxy) by Mazenkovic
  __  __                       ____              _                _ 
 |  \/  | __ _ _______ _ __   |  _ \  __ _  __ _| |__   __ _ _ __(_)
 | |\/| |/ _` |_  / _ \ '_ \  | | | |/ _` |/ _` | '_ \ / _` | '__| |
 | |  | | (_| |/ /  __/ | | | | |_| | (_| | (_| | | | | (_| | |  | |
 |_|  |_|\__,_/___\___|_| |_| |____/ \__,_|\__, |_| |_|\__,_|_|  |_|
                                           |___/                    
 4-Wheel Drive Mobile Robot Simulation built in ROS2 Foxy, featuring:

     _____   __ _____  _______ ______       ___                                            __              
    |     |_|__|     \|   _   |   __ \    .'  _.-----.----.    .--------.---.-.-----.-----|__.-----.-----. 
    |       |  |  --  |       |      <    |   _|  _  |   _|    |        |  _  |  _  |  _  |  |     |  _  | 
    |_______|__|_____/|___|___|___|__|    |__| |_____|__|      |__|__|__|___._|   __|   __|__|__|__|___  | 
                                                                              |__|  |__|           |_____| 
                __            __          __              __                                               
.---.-.-----.--|  |    .-----|  |--.-----|  |_.---.-.----|  .-----.                                        
|  _  |     |  _  |    |  _  |  _  |__ --|   _|  _  |  __|  |  -__|                                        
|___._|__|__|_____|    |_____|_____|_____|____|___._|____|__|_____|                                        
                                                                                                           
    __       __              __   __                                                                       
.--|  .-----|  |_.-----.----|  |_|__.-----.-----.                                                          
|  _  |  -__|   _|  -__|  __|   _|  |  _  |     |                                                          
|_____|_____|____|_____|____|____|__|_____|__|__|                                                          
                                                                                                           
     _____              __   __         ______                                       ___                   
    |     \.-----.-----|  |_|  |--.    |      .---.-.--------.-----.----.---.-.    .'  _.-----.----.       
    |  --  |  -__|  _  |   _|     |    |   ---|  _  |        |  -__|   _|  _  |    |   _|  _  |   _|       
    |_____/|_____|   __|____|__|__|    |______|___._|__|__|__|_____|__| |___._|    |__| |_____|__|         
                 |__|                                                                                      
       __       __                    __                         __                                        
.--.--|__.-----|__.-----.-----.______|  |--.---.-.-----.-----.--|  |                                       
|  |  |  |__ --|  |  _  |     |______|  _  |  _  |__ --|  -__|  _  |                                       
 \___/|__|_____|__|_____|__|__|      |_____|___._|_____|_____|_____|                                       
                                                                                                           
                   __ __            __   __                                                                
.---.-.-----.-----|  |__.----.---.-|  |_|__.-----.-----.-----.                                             
|  _  |  _  |  _  |  |  |  __|  _  |   _|  |  _  |     |__ --|                                             
|___._|   __|   __|__|__|____|___._|____|__|_____|__|__|_____|                                             
      |__|  |__|                                                                                           
     _______       __                                    __   __                   ___                     
    |_     _.-----|  .-----.-----.-----.-----.----.---.-|  |_|__.-----.-----.    .'  _.-----.----.         
      |   | |  -__|  |  -__|  _  |  _  |  -__|   _|  _  |   _|  |  _  |     |    |   _|  _  |   _|         
      |___| |_____|__|_____|_____|   __|_____|__| |___._|____|__|_____|__|__|    |__| |_____|__|           
                                 |__|                                                                      
                                  __                       __              __                              
.--------.---.-.-----.--.--.---.-|  |    .----.-----.-----|  |_.----.-----|  |                             
|        |  _  |     |  |  |  _  |  |    |  __|  _  |     |   _|   _|  _  |  |                             
|__|__|__|___._|__|__|_____|___._|__|    |____|_____|__|__|____|__| |_____|__|                             
                                                                                                           
     _______       __                                                                                      
    |   _   .--.--|  |_.-----.-----.-----.--------.-----.--.--.-----.                                      
    |       |  |  |   _|  _  |     |  _  |        |  _  |  |  |__ --|                                      
    |___|___|_____|____|_____|__|__|_____|__|__|__|_____|_____|_____|                                      
                                                                                                           
 _______             __             __   __                   ___ _______             ______ ___           
|    |  .---.-.--.--|__.-----.---.-|  |_|__.-----.-----.    ,'  _|    |  .---.-.--.--|__    |_  `.         
|       |  _  |  |  |  |  _  |  _  |   _|  |  _  |     |    |  | |       |  _  |  |  |    __| |  |         
|__|____|___._|\___/|__|___  |___._|____|__|_____|__|__|    |  |_|__|____|___._|\___/|______|_|  |         
                       |_____|                              `.___|                          |___,'         
  ___                             __   __               __                   __                            
.'  _.-----.----.    .-----.---.-|  |_|  |--.    .-----|  .---.-.-----.-----|__.-----.-----.               
|   _|  _  |   _|    |  _  |  _  |   _|     |    |  _  |  |  _  |     |     |  |     |  _  |               
|__| |_____|__|      |   __|___._|____|__|__|    |   __|__|___._|__|__|__|__|__|__|__|___  |               
                     |__|                        |__|                                |_____|               
     _______                   __               _______ __                __       __   __                 
    |     __.---.-.-----.-----|  |--.-----.    |     __|__.--------.--.--|  .---.-|  |_|__.-----.-----.    
    |    |  |  _  |-- __|  -__|  _  |  _  |    |__     |  |        |  |  |  |  _  |   _|  |  _  |     |    
    |_______|___._|_____|_____|_____|_____|    |_______|__|__|__|__|_____|__|___._|____|__|_____|__|__|    
                                                                                                           
  ___                                  __ __       __   __                 __                __            
.'  _.-----.----.    .----.-----.---.-|  |__.-----|  |_|__.----.    .-----|  |--.--.--.-----|__.----.-----.
|   _|  _  |   _|    |   _|  -__|  _  |  |  |__ --|   _|  |  __|    |  _  |     |  |  |__ --|  |  __|__ --|
|__| |_____|__|      |__| |_____|___._|__|__|_____|____|__|____|    |   __|__|__|___  |_____|__|____|_____|
                                                                    |__|        |_____|                    
Features
🔹 Gazebo Simulation – Fully simulated environment for robot testing
🔹 SLAM & Navigation – Autonomous mapping and path planning with Nav2
🔹 Sensor Suite – Includes LiDAR, IMU, depth camera, ultrasonic sensors
🔹 Teleoperation – Control via keyboard or joystick
🔹 4WD Differential Drive – Supports skid steering and Ackermann steering
🔹 ROS2 Nodes – Modular design with separate nodes for perception, control, and navigation

![slam](https://github.com/mazen-daghari/mazeN/blob/334f24de1bf3aeb0b9ea591387faadd7f677bd82/slam.gif)

![Robot Moving](https://github.com/mazen-daghari/mazeN/blob/753a22411766561b1b7b400f00673db399df9285/moving_robot.gif)

--


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

--

for slam 

ros2 launch slam_toolbox online_async_launch.py params_file:=$(pwd)/src/mazeN/config/mapper_params_online_async.yaml use_sim_time:=true

for amcl 

ros2 run nav2_amcl amcl --ros-args -p use_sim_time:=true

--

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

-

License
📜 GPL3
feel free to contact me dagmazen@gmail.com
