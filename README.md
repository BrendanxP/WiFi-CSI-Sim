# WiFi-CSI-Sim
WiFi-CSI Bearing Estimation and Localization in Multi-Robot Systems: An Open-Source Simulation Framework

The framework handles the ROS setup, spawning robots at desired positions (and height), designing trajectories and experimental setups, simulating the CSI signal behavior in real-time, and integration with applications usch as signal AoA profile generation, bearing estimation and localization. Moreover, it can easily be expanded and adapted to fit the users needs.

The simulation framework relies on MATLAB with Gazebo and ROS. The framework can be run on a single Linux machine, or can be distributed over two PC's where the Gazebo simulator requires a Linux environment and the Controller software with MATLAB can be deployed on either Linux or Windows. The most logical options are as follows, where we prefer option 1 combining the strengths of Linux and Windows:
1. Gazebo simulator (PC1: Linux) + Controller (PC2: Linux/Windows)
2. Gazebo simulator (PC1: Linux) + Contoller (PC1: Linux)
3. Gazebo simulator (PC1: VM Linux) + Controller (PC1: Windows)

All configuration settings are performed in the MALTAB Live Script MRS25.m, including the details to setup the connection to Gazebo, which are further elaborated on below at **Set-up steps**.


**Prerequisites**

Gazebo simulator (Linux) _(can be the same device as the Controller (linux) or a VM on the Controller (Windows))_:

1. ROS1 Noetic with Gazebo installed on Ubuntu 20.04 (tested with ROS1 Noetic, slight adaptations could theoretically make it work with ROS2 but this has yet to be tested).
2. Catkin workspace with the required robotic platforms loaded and GAZEBO_MODEL_PATH, ROS_PACKAGE_PATH and GAZEBO_PLUGIN_PATH properly set in .bashrc, corresponding to the requirements of the robotic platform, 
3. The robotic platforms tested inlcude the following, but it should allow for any robot that can be spawned using SDF or URDF:
- Turtlebot3 Waffle Pi https://github.com/ROBOTIS-GIT/turtlebot3
- Nexus 4WD Mecanum https://github.com/RBinsonB/nexus_4wd_mecanum_simulator 

Controller (Windows/Linux) _(can be the same device as the Gazebo simulator)_:

1. MALTAB, tested with 2024b or 2025a,
2. MATLAB ROS Toolbox Add-on incl the required Python installation,
3. The MATLAB code files to run the experiment from this repository,
4. The URDF or SDF files for the robots to be spawned with, which must contain valid path diretories to files on the Linux PC using GAZEBO_MODEL_PATH, ROS_PACKAGE_PATH and GAZEBO_PLUGIN_PATH.


**Set-up steps**

Gazebo simulator:

1. First have all prerequisites installed and the proper bashrc set regarding the ROBOTIC PLATFORM, otherwise robots will not spawn properly (missing graphics and/or controller plugins). Similarly, setup the Custom CSI ROS message as shown below.
2. Make sure the Gazebo simulator and Controller are on the same network and find the IPs.
3. Using these IP's set the ROS_IP, ROS_HOSTNAME, and ROS_MASTER_URI accordingly in the bashrc on the Gazebo simulator. You can opt to run the ROS Core on the Gazebo simulator (manually start in a terminal window) or on the Controller where it is togglable in the MALTAB Live script along with fields to set the IPs in here.
4. Using a second terminal on the Gazebo simulator, one can start Gazebo with or without UI depending on the requirements. Here, any world (without robots) can be launched, but to get a simple empty world the following commands can be run:
- rosrun gazebo_ros gazebo (empty world, connected to ROS, with GUI)
- roslaunch gazebo_ros empty_world.launch gui:=false (empty world, connected to ROS, no GUI)

Custom CSI ROS messages:

-> On Gazebo Simulator

## Creating the `csi_msgs` Custom Message Package for ROS

Below is a concise, copy-ready overview to include in your Github instructions. This guides users in setting up the `csi_msgs` package for custom CSI messages, without affecting other packages in the workspace.

### 1. Create the ROS Package

From your workspace source folder:
```bash
cd ~/catkin_ws/src
catkin_create_pkg csi_msgs std_msgs message_generation
cd csi_msgs
```

### 2. Define Custom Messages

Make a `msg/` directory if it does not exist:
```bash
mkdir msg
```

In `msg/`, create two files:

**CsiForNode.msg**
```msg
string target_node
uint32 packet_number
std_msgs/Float64MultiArray csi_data
```

**CsiBundle.msg**
```msg
csi_msgs/CsiForNode[] bundle
```

### 3. Edit CMakeLists.txt

Ensure these sections are included and properly set:

```cmake
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)
add_message_files(
  FILES
  CsiForNode.msg
  CsiBundle.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)
```

### 4. Edit package.xml

Add or confirm the following dependencies inside ``:

```xml
message_generation
message_runtime
std_msgs
std_msgs
```

### 5. Build the Package

Return to your workspace root, then build:
```bash
cd ~/catkin_ws
catkin_make
```

### 6. Source the Workspace

After building, source your workspace setup script:
```bash
source ~/catkin_ws/devel/setup.bash
```

_Optional:_ Add this command to your `~/.bashrc` to automate sourcing.

### 7. Verify the Messages

Check that your new messages are available:

```bash
rosmsg show csi_msgs/CsiForNode
rosmsg show csi_msgs/CsiBundle
```

-> On MATLAB Controller (only required if using Windows)
1. Copy the csi_msgs folder from Ubuntu to your Windows device.
2. run ``rosgenmsg("C:\the-parent-folder-of-csi_msgs")``
3. If you encounter an error about compilation make sure to have the VScode compiler installed: https://visualstudio.microsoft.com/vs/community/ . Select "Desktop development with C++" workload. After installation run ``mex -setup cpp``.


MATLAB Controller:

With the Gazebo simulator setup properly, the Live Script in MATLAB can be configured and used to start the experiments.

1. Set the IP configuration of the two nodes on the network and initiate ROS on MALTAB.
2. Select the robot files of your chosing, the amount of robots, their spawn location and MAC address identiefier. Once run, the robots should spawn 1 by 1 into the world.

Everything up and untill here only needs to be run once to be able to do repeated experiments using the confgured layout of the robots.

3. This step involves most control options in the MATLAB Live Script. Here, the experiment duration, robot trajectories, communications links, signal properties (Channel, Number of subcarriers, SNR, CFO), and more can be configured with easy configurable toggles. When everything is set, you can hit run and on the section and the single experiment will start. 
4. After the experiment completes a directory can be selected where the results can be written to. This includes figures on carrier signal, the driven odometries, AoA profiles for each connection, and a localization overview. 
5. Alternatively a Monte Carlo setup is made which automatically runs an X number of repetitions and swiftly processes the bulk results into results and insights.

The framework includes the required backbone which can easily be adapted or upgraded to fit to specific purposes and goals.
