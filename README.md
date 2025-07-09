# WiFi-CSI-Sim
WiFi-CSI Bearing Estimation and Localization in Multi-Robot Systems: An Open-Source Simulation Framework

The simulation framework relies on MATLAB with Gazebo and ROS. The framework can be run on a single Linux machine, or can be distributed over a Linux based Gazebo simulator and a Windows or Linux controlling node running MALTAB. All configuration settings are performed in the MALTAB Live Script MRS25.m, including the details to setup the connection to Gazebo.


**Prerequisites**
Gazebo simulator (Linux) _(can be the same device as the Controller (linux) or a VM on the Controller (Windows))_:
ROS1 with Gazebo installed (tested with ROS1 Noetic, slight adaptations could theoretically make it work with ROS2 but this has yet to be tested).
Catkin workspace with the required robotic platforms loaded and GAZEBO_MODEL_PATH, ROS_PACKAGE_PATH and GAZEBO_PLUGIN_PATH properly set in .bashrc, corresponding to the requirements of the robotic platform, 
The robots tested with inlcude the following, but it should allow for any robot that can be spawned using SDF or URDF:
- Turtlebot3 https://github.com/ROBOTIS-GIT/turtlebot3
- Nexus 4WD Mecanum https://github.com/RBinsonB/nexus_4wd_mecanum_simulator 

Controller (Windows/Linux) _(can be the same device as the Gazebo simulator)_:
MALTAB, tested with 2024b or 2025a,
MATLAB ROS Toolbox Add-on incl the required Python installation,
The MATLAB code files to run the experiment from this repository,
The URDF or SDF files for the robots to be spawned.



**Set-up steps**
Gazebo simulator:
First have all prerequisites installed and the proper bashrc set regarding the ROBOTIC PLATFORM, otherwise robots will not spawn properly (missing graphics and/or controller plugins).
Make sure the Gazebo simulator and Controller are on the same network and find the IPs.
Using these IP's set the ROS_IP, ROS_HOSTNAME, and ROS_MASTER_URI accordingly in the bashrc on the Gazebo simulator. You can opt to run the ROS Core on the Gazebo simulator (manually start in a terminal window) or on the Controller where it is togglable in the MALTAB Live script along with fields to set the IPs in here.
Using a second terminal on the Gazebo simulator, one can start Gazebo with or without UI depending on the requirements. Here, any world (without robots) can be launched, but to get a simple empty world the following commands can be run:
- rosrun gazebo_ros gazebo (empty world, connected to ROS, with GUI)
- roslaunch gazebo_ros empty_world.launch gui:=false (empty world, connected to ROS, no GUI)

MATLAB
With the Gazebo simulator setup properly, the Live Script in MATLAB can be configured and used to start the experiments.
1. Set the IP configuration of the two nodes on the network and initiate ROS on MALTAB.
2. Select the robot files of your chosing, the amount of robots, their spawn location and MAC address identiefier. Once run, the robots should spawn 1 by 1 into the world.

Everything up and untill here only needs to be run once to be able to do repeated experiments using the confgured layout of the robots.
3. This step involves most control options in the MATLAB Live Script. Here, the experiment duration, robot trajectories, communications links, signal properties (Channel, Number of subcarriers, SNR, CFO), and more can be configured with easy configurable toggles. When everything is set, you can hit run and on the section and the single experiment will start. 
4. After the experiment completes a directory can be selected where the results can be written to. This includes figures on carrier signal, the driven odometries, AoA profiles for each connection, and a localization overview. 

5. Alternatively a Monte Carlo setup is made which automatically runs an X number of repetitions
