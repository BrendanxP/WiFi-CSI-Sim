%[text] # Gazebo/ROS Nexus Robot Simulation
%[text] #### Clear, Close and Kill ROS
  %[control:button:61bf]{"position":[1,2]}
KillROS() % Confirm in chat to prevent accidental data-loss
%%
%[text] ## Initialize ROS in MATLAB
%[text] Define node IPs and select ROS Core location.
  %[control:button:47b0]{"position":[1,2]}
conf.LocalIP =      '192.168.137.1';  % Windows device IP (Linux possible) %[control:editfield:09c4]{"position":[21,36]}
conf.RemoteIP =     '192.168.137.98';  % Linux device IP 218 %[control:editfield:6987]{"position":[21,37]}
conf.RemoteUser =   "brendan";  % Linux device username % only needed for drone spawn %[control:editfield:72e4]{"position":[21,30]}
conf.RosCoreLocal = false;  % Location of ROS Core %[control:dropdown:66ef]{"position":[21,26]}
master = InitROS(conf); %[output:60ae8474]
%[text] rosrun gazebo\_ros gazebo (empty world, connected to ROS, **with GUI**)
%[text] roslaunch gazebo\_ros empty\_world.launch gui:=false (empty world, connected to ROS, **no GUI**)
%%
%[text] ## Robot Setup Configuration
%[text] Here we spawn the robots in Gazebo and create the interface to interact with them.
  %[control:button:290e]{"position":[1,2]}
debug.ros =   false; % Rosnodes and topics info %[control:checkbox:2e89]{"position":[15,20]}
debug.nexus = false; % Rostopic info on Nexus Robot %[control:checkbox:81d8]{"position":[15,20]}

% Path of Nexus Robot File
%sdf_path_ground = pwd + "\nexus_robot2.sdf";
sdf_path_ground = "C:\Users\b.dijkstra\OneDrive - University of Groningen\Documenten\Industrial Engineering & Management\2.0 MSc Industrial Engineering & Management\4.2 Research Project - WiFi-CSI-Sensing\_ICRA25\MATLAB\Final code\waffle_pi.urdf"; % Select ground robot SDF or URDF file %[control:filebrowser:0ba9]{"position":[19,247]}
sdf_path_drone = "pwd + '\aerial_robot.urdf'"; %[control:filebrowser:9739]{"position":[18,46]}


% Number of Receivers
rx_count =6; %[control:slider:8e86]{"position":[11,12]}
R = struct('i', num2cell(1:rx_count)); % predefine struct size
clearvars rx_count; 

% Start positions of TX and RX (x,y,z [m], azimuth [deg])
posStart = { ...
    [ -5.62, 0.06, 1, 0 ]; % R1 %[control:editfield:9f3f]{"position":[5,26]}
    [ 5.15, 2.5, 2, 0 ]; % R2 %[control:editfield:1727]{"position":[5,24]}
    [ 5.15, -1.8, 3, 0 ]; % R3 %[control:editfield:6abd]{"position":[5,25]}
    [0.18, 2.12, 0, 0]; % R4 %[control:editfield:8317]{"position":[5,23]}
    [1,-1,0,0]; % R5 %[control:editfield:21ed]{"position":[5,15]}
    [ -3, -2, 1.5, 0 ]; % R6 %[control:editfield:35eb]{"position":[5,23]}
    [1, -3, 0, 0 ]; % R7 %[control:editfield:5f0d]{"position":[5,19]}
    [ 8, 8, 2, 0 ]; % R8 %[control:editfield:90eb]{"position":[5,19]}
    [9, 9, 3, 0]; % R9 %[control:editfield:6ae6]{"position":[5,17]}
    [10,10,4,0]; % R10 %[control:editfield:7d90]{"position":[5,16]}
};
[R.posStart] = posStart{:}; clearvars posStart;

% 2d vs 3d, ground robot or aerial drone
is3D = { ...
    0; % R1 %[control:dropdown:0c8d]{"position":[5,6]}
    0; % R2  %[control:dropdown:4361]{"position":[5,6]}
    0; % R3 %[control:dropdown:7f68]{"position":[5,6]}
    0; % R4 %[control:dropdown:4041]{"position":[5,6]}
    0; % R5 %[control:dropdown:69ce]{"position":[5,6]}
    0; % R6 %[control:dropdown:1d17]{"position":[5,6]}
    0; % R7  %[control:dropdown:486c]{"position":[5,6]}
    0; % R8 %[control:dropdown:99a5]{"position":[5,6]}
    0; % R9 %[control:dropdown:3ad9]{"position":[5,6]}
    0; % R10 %[control:dropdown:1fdc]{"position":[5,6]}
}; 
[R.is3D] = is3D{:}; clearvars is3D;

% List of MAC address unique identifier of the nodes.
MAC = { ...
    '11111111AA'; % R1 %[control:editfield:11d2]{"position":[5,17]}
    '22222222BB'; % R2 %[control:editfield:2731]{"position":[5,17]}
    '33333333CC'; % R3 %[control:editfield:9b1d]{"position":[5,17]}
    '44444444DD'; % R4 %[control:editfield:3834]{"position":[5,17]}
    '55555555EE'; % R5 %[control:editfield:593f]{"position":[5,17]}
    '66666666FF'; % R6 %[control:editfield:7e71]{"position":[5,17]}
    '77777777GG'; % R7 %[control:editfield:9279]{"position":[5,17]}
    '88888888HH'; % R8 %[control:editfield:958e]{"position":[5,17]}
    '99999999II'; % R9 %[control:editfield:6122]{"position":[5,17]}
    '00000000JJ'; % R10 %[control:editfield:9688]{"position":[5,17]}
}; 
[R.MAC] = MAC{:}; clearvars MAC;


% Spawn Nexus Robots in Gazebo
[rosSub,rosPub,msgPub,cltArming,cltSetMode]=rosSpawn(sdf_path_ground,sdf_path_drone,R,debug,conf); %[output:2e96ce54]
[R.rosSub] = rosSub{:}; clearvars rosSub;
[R.rosPub] = rosPub{:}; clearvars rosPub;
[R.msgPub] = msgPub{:}; clearvars msgPub;
[R.cltSetMode] = cltSetMode{:}; clearvars cltSetMode;
[R.cltArming] = cltArming{:}; clearvars cltArming;
clearvars df_path_ground sdf_path_drone;

% Clients to interact with ROS/Gazebo simulation
pauseClient = rossvcclient('/gazebo/pause_physics'); % Pause and unpause e.g. to do heavy calculations during run
unpauseClient = rossvcclient('/gazebo/unpause_physics');
resetWorld = rossvcclient('/gazebo/reset_world'); % resets robots, not time
resetSimulation = rossvcclient('/gazebo/reset_simulation'); % reset robots and time
physicsClient = rossvcclient('/gazebo/set_physics_properties'); % Set Gazebo simulation time to slowdown for reaching desired rate
getPhysicsClient = rossvcclient('/gazebo/get_physics_properties');
simPhysics = call(getPhysicsClient); % Get current physics properties
%%
%[text] ## Experiment Configuration
%[text] Here we define all settings for the experiment using the already spawned robots.
%[text] #### SIGNAL PARAMETERS
% See channelFrequency() for all options
channel = 108; %[control:editfield:8f87]{"position":[11,14]}
numSubC = 30; %[control:editfield:2a2f]{"position":[11,13]}
[centerFrequency,bandwidth] = channelFrequency(channel);
lambda = physconst('LightSpeed')/centerFrequency;
%dataFormat = "Nexmon"; %[control:dropdown:709c]{"position":[15,23]}
ros_rate =   100; % This is the preset ROS rate, keep in mind this is not always achieved [Hz] %[control:editfield:7e54]{"position":[14,17]}
num_points = 200; %[control:editfield:86b4]{"position":[14,17]}
% Set ROS rate
rate = rosrate(ros_rate); % rate based on simulation time
rate.OverrunAction='slip'; % 'slip' immediately execute loop again when late, otherwise 'drop'

% Configure new physics properties to get best performance
simPhysics.MaxUpdateRate = 1000;    % Default: 1000 [Hz] max internal update rate
simPhysics.TimeStep = 1e-3;         % Default 1e-3 [s] timestep

% Apply new settings
response = call(physicsClient, simPhysics);
if ~response.Success
    p_error('Failed to adjust simulation speed');
end
%[text] #### ROBOT TRAJECTORIES
  %[control:button:0938]{"position":[1,2]}
% Communication between robots
%            |1| |2| |3| |4| |5| |6| |7| |8| |9|  
comMatrix = [false, 0,  0,  0,  0,  0,  0,  0,  0;  % R2 %[control:checkbox:1e72]{"position":[14,19]}
             false,false, 0,  0,  0,  0,  0,  0,  0;  % R3 %[control:checkbox:83d3]{"position":[14,19]} %[control:checkbox:3507]{"position":[20,25]}
             true,true,true, 0,  0,  0,  0,  0,  0;  % R4 %[control:checkbox:1cdb]{"position":[14,18]} %[control:checkbox:8506]{"position":[19,23]} %[control:checkbox:7236]{"position":[24,28]}
             true,true,true,true, 0,  0,  0,  0,  0;  % R5 %[control:checkbox:89b7]{"position":[14,18]} %[control:checkbox:3e66]{"position":[19,23]} %[control:checkbox:9ec1]{"position":[24,28]} %[control:checkbox:8d12]{"position":[29,33]}
             false,false,false,true,true, 0,  0,  0,  0;  % R6 %[control:checkbox:86ff]{"position":[14,19]} %[control:checkbox:33d6]{"position":[20,25]} %[control:checkbox:7995]{"position":[26,31]} %[control:checkbox:4b98]{"position":[32,36]} %[control:checkbox:683c]{"position":[37,41]}
             false,false,false,false,false,false, 0,  0,  0;  % R7 %[control:checkbox:1f24]{"position":[14,19]} %[control:checkbox:6cd0]{"position":[20,25]} %[control:checkbox:6afb]{"position":[26,31]} %[control:checkbox:19ff]{"position":[32,37]} %[control:checkbox:9028]{"position":[38,43]} %[control:checkbox:8766]{"position":[44,49]}
             false,false,false,false,false,false,false, 0,  0;  % R8 %[control:checkbox:7809]{"position":[14,19]} %[control:checkbox:40a3]{"position":[20,25]} %[control:checkbox:518b]{"position":[26,31]} %[control:checkbox:1427]{"position":[32,37]} %[control:checkbox:1e5e]{"position":[38,43]} %[control:checkbox:9b4d]{"position":[44,49]} %[control:checkbox:23a7]{"position":[50,55]}
             false,false,false,false,false,false,false,false, 0;  % R9 %[control:checkbox:8aca]{"position":[14,19]} %[control:checkbox:5f2f]{"position":[20,25]} %[control:checkbox:3a2a]{"position":[26,31]} %[control:checkbox:7a78]{"position":[32,37]} %[control:checkbox:261d]{"position":[38,43]} %[control:checkbox:5457]{"position":[44,49]} %[control:checkbox:84af]{"position":[50,55]} %[control:checkbox:61c4]{"position":[56,61]}
             false,false,false,false,false,false,false,false,false]; % R10 %[control:checkbox:8498]{"position":[14,19]} %[control:checkbox:21fb]{"position":[20,25]} %[control:checkbox:6a6b]{"position":[26,31]} %[control:checkbox:09ff]{"position":[32,37]} %[control:checkbox:1297]{"position":[38,43]} %[control:checkbox:35e7]{"position":[44,49]} %[control:checkbox:71f9]{"position":[50,55]} %[control:checkbox:29d7]{"position":[56,61]} %[control:checkbox:8eb1]{"position":[62,67]}

% Translate comMatrix to communication array per node
comMatrix = comMatrix(1:numel(R)-1,1:numel(R)-1);
temp = false(numel(R),numel(R));
temp(2:end,1:end-1) = comMatrix;
temp(1:end-1,2:end) = logical(comMatrix' + temp(1:end-1,2:end));
temp = mat2cell(temp,[ones(1,numel(R))]);
[R.comDevices] = temp{:}; clearvars temp;

% specify circle, sin-wave, line, rendezvous (follow a selected robot)
trajTypeOptions = ["static","circle","sin-wave","line","rendezvous","random-walk"];
trajType = { ...    
    trajTypeOptions(1); % R1 %[control:dropdown:9d43]{"position":[5,23]}
    trajTypeOptions(1); % R2 %[control:dropdown:83f2]{"position":[5,23]}
    trajTypeOptions(1); % R3 %[control:dropdown:09a6]{"position":[5,23]}
    trajTypeOptions(6); % R4 %[control:dropdown:2357]{"position":[5,23]}
    trajTypeOptions(6); % R5 %[control:dropdown:4e01]{"position":[5,23]}
    trajTypeOptions(1); % R6 %[control:dropdown:0bcb]{"position":[5,23]}
    trajTypeOptions(1); % R7 %[control:dropdown:8bfb]{"position":[5,23]}
    trajTypeOptions(1); % R8 %[control:dropdown:9930]{"position":[5,23]}
    trajTypeOptions(1); % R9 %[control:dropdown:6e23]{"position":[5,23]}
    trajTypeOptions(1); % R10 %[control:dropdown:48a8]{"position":[5,23]}
}; 
[R.trajType] = trajType{:}; clearvars trajType;

% if rendezvous, select which robot to follow
robotList = 1:numel(R);
trajRDVTarget = { ...
    robotList(1); % R1 %[control:dropdown:057b]{"position":[5,17]}
    robotList(1); % R2 %[control:dropdown:936a]{"position":[5,17]}
    robotList(1); % R3 %[control:dropdown:2009]{"position":[5,17]}
    robotList(1); % R4 %[control:dropdown:4f44]{"position":[5,17]}
    robotList(1); % R5 %[control:dropdown:9b2e]{"position":[5,17]}
    robotList(1); % R6 %[control:dropdown:0bf8]{"position":[5,17]}
    robotList(1); % R7 %[control:dropdown:4738]{"position":[5,17]}
    robotList(1); % R8 %[control:dropdown:5c11]{"position":[5,17]}
    robotList(1); % R9 %[control:dropdown:277c]{"position":[5,17]}
    robotList(1); % R10 %[control:dropdown:12a9]{"position":[5,17]}
}; 
[R.trajRDVTarget] = trajRDVTarget{:}; clearvars trajRDVTarget;

% Omnidirection or differential
trajOmni = { ...
    false; % R1 %[control:checkbox:7c51]{"position":[5,10]}
    false; % R2 %[control:checkbox:22ab]{"position":[5,10]}
    false; % R3 %[control:checkbox:878e]{"position":[5,10]}
    false; % R4 %[control:checkbox:11b5]{"position":[5,10]}
    false; % R5 %[control:checkbox:8188]{"position":[5,10]}
    false; % R6 %[control:checkbox:974f]{"position":[5,10]}
    false; % R7 %[control:checkbox:8284]{"position":[5,10]}
    false; % R8 %[control:checkbox:2d71]{"position":[5,10]}
    false; % R9 %[control:checkbox:2eb1]{"position":[5,10]}
    false; % R10 %[control:checkbox:903a]{"position":[5,10]}
}; 
[R.trajOmni] = trajOmni{:}; clearvars trajOmni;


%% Omnidirectional rotation
% Radius of circle, radius of half circles in sin wave, half line length [m]
trajRad  = { ...
    0; % R1 %[control:editfield:404b]{"position":[5,6]}
    0; % R2 %[control:editfield:4f4f]{"position":[5,6]}
    0; % R3 %[control:editfield:9b11]{"position":[5,6]}
    0.4; % R4 %[control:editfield:4f42]{"position":[5,8]}
    0.1; % R5 %[control:editfield:28c3]{"position":[5,8]}
    0; % R6 %[control:editfield:18b8]{"position":[5,6]}
    0.2; % R7 %[control:editfield:1a33]{"position":[5,8]}
    0; % R8 %[control:editfield:574a]{"position":[5,6]}
    0; % R9 %[control:editfield:391d]{"position":[5,6]}
    0; % R10 %[control:editfield:372a]{"position":[5,6]}
}; 
[R.trajRad] = trajRad{:}; clearvars trajRad;

%% Differential rotation
% Rotational speed robot [rad/s]
trajRot  = { ...
    0; % R1 %[control:editfield:1994]{"position":[5,6]}
    0; % R2 %[control:editfield:27ea]{"position":[5,6]}
    0; % R3 %[control:editfield:1890]{"position":[5,6]}
    0.4; % R4 %[control:editfield:6b17]{"position":[5,8]}
    0.4; % R5 %[control:editfield:2e35]{"position":[5,8]}
    0; % R6 %[control:editfield:5ad3]{"position":[5,6]}
    0.2; % R7 %[control:editfield:449c]{"position":[5,8]}
    0; % R8 %[control:editfield:35f4]{"position":[5,6]}
    0; % R9 %[control:editfield:1ec2]{"position":[5,6]}
    0; % R10 %[control:editfield:4431]{"position":[5,6]}
}; 
[R.trajRot] = trajRot{:}; clearvars trajRot;

% Magnitude of velocity as sqrt(x^2+y^2), so same speed on any angle [m/s]
trajVel  = { ...
    0; % R1 %[control:editfield:259d]{"position":[5,6]}
    0; % R2 %[control:editfield:6bb0]{"position":[5,6]}
    0; % R3 %[control:editfield:6d07]{"position":[5,6]}
    0.04; % R4 %[control:editfield:00ff]{"position":[5,9]}
    0.1; % R5 %[control:editfield:405f]{"position":[5,8]}
    0; % R6 %[control:editfield:7ee9]{"position":[5,6]}
    0.01; % R7 %[control:editfield:591c]{"position":[5,9]}
    0; % R8 %[control:editfield:10f6]{"position":[5,6]}
    0; % R9 %[control:editfield:1e8c]{"position":[5,6]}
    0; % R10 %[control:editfield:472b]{"position":[5,6]}
}; 
[R.trajVel] = trajVel{:}; clearvars trajVel;

% Delta Frequency for Channel Frequency Offset [Hz] of the robots antenna
deltaFrequency = { ...
    941489; % R1 %[control:editfield:6a3f]{"position":[5,11]}
    501493; % R2 %[control:editfield:2cb3]{"position":[5,11]}
    393073; % R3 %[control:editfield:1330]{"position":[5,11]}
    471091; % R4 %[control:editfield:7292]{"position":[5,11]}
    850081; % R5 %[control:editfield:0d54]{"position":[5,11]}
    659159; % R6 %[control:editfield:5866]{"position":[5,11]}
    354001; % R7 %[control:editfield:8f5f]{"position":[5,11]}
    942311; % R8 %[control:editfield:824c]{"position":[5,11]}
    525719; % R9 %[control:editfield:0dd7]{"position":[5,11]}
    325631; % R10 %[control:editfield:35f1]{"position":[5,11]}
}; 
[R.deltaFrequency] = deltaFrequency{:}; clearvars deltaFrequency;

% Signal-to-noise ratio for Gaussian white noise [-]
% We define two levels of SNR, high or low depening on LOS or NLOS
SNRhigh =    5; % Higher figure is less noise (LOS) %[control:editfield:8d21]{"position":[14,15]}
SNRlow =     3; % Lower figure is more noise (NLOS) %[control:editfield:25cb]{"position":[14,15]}

% Select connections with high/LOS (true) or low/NLOS (false) SNR
%            |1| |2| |3| |4| |5| |6| |7| |8| |9|  
SNRMatrix = [true, 0,  0,  0,  0,  0,  0,  0,  0;  % R2 %[control:checkbox:2135]{"position":[14,18]}
             true,false, 0,  0,  0,  0,  0,  0,  0;  % R3 %[control:checkbox:42fc]{"position":[14,18]} %[control:checkbox:24b0]{"position":[19,24]}
             true,false,false, 0,  0,  0,  0,  0,  0;  % R4 %[control:checkbox:2912]{"position":[14,18]} %[control:checkbox:49bb]{"position":[19,24]} %[control:checkbox:38ca]{"position":[25,30]}
             true,false,false,false, 0,  0,  0,  0,  0;  % R5 %[control:checkbox:9f3f]{"position":[14,18]} %[control:checkbox:5e43]{"position":[19,24]} %[control:checkbox:5256]{"position":[25,30]} %[control:checkbox:3978]{"position":[31,36]}
             true,true,true,true,true, 0,  0,  0,  0;  % R6 %[control:checkbox:0277]{"position":[14,18]} %[control:checkbox:91c3]{"position":[19,23]} %[control:checkbox:8c8d]{"position":[24,28]} %[control:checkbox:8389]{"position":[29,33]} %[control:checkbox:7c16]{"position":[34,38]}
             true,false,false,false,false,true, 0,  0,  0;  % R7 %[control:checkbox:2088]{"position":[14,18]} %[control:checkbox:8325]{"position":[19,24]} %[control:checkbox:3050]{"position":[25,30]} %[control:checkbox:759b]{"position":[31,36]} %[control:checkbox:5e84]{"position":[37,42]} %[control:checkbox:74f5]{"position":[43,47]}
             true,false,false,false,false,true,false, 0,  0;  % R8 %[control:checkbox:08c5]{"position":[14,18]} %[control:checkbox:8e20]{"position":[19,24]} %[control:checkbox:7b49]{"position":[25,30]} %[control:checkbox:5443]{"position":[31,36]} %[control:checkbox:67f7]{"position":[37,42]} %[control:checkbox:6784]{"position":[43,47]} %[control:checkbox:0620]{"position":[48,53]}
             true,false,false,false,false,true,false,false, 0;  % R9 %[control:checkbox:4b56]{"position":[14,18]} %[control:checkbox:5b42]{"position":[19,24]} %[control:checkbox:38ab]{"position":[25,30]} %[control:checkbox:7e4a]{"position":[31,36]} %[control:checkbox:43b6]{"position":[37,42]} %[control:checkbox:33c2]{"position":[43,47]} %[control:checkbox:8685]{"position":[48,53]} %[control:checkbox:97c4]{"position":[54,59]}
             true,false,false,false,false,true,false,false,false]; % R10 %[control:checkbox:9785]{"position":[14,18]} %[control:checkbox:117a]{"position":[19,24]} %[control:checkbox:1b22]{"position":[25,30]} %[control:checkbox:6e04]{"position":[31,36]} %[control:checkbox:66b3]{"position":[37,42]} %[control:checkbox:0324]{"position":[43,47]} %[control:checkbox:574c]{"position":[48,53]} %[control:checkbox:6c08]{"position":[54,59]} %[control:checkbox:0546]{"position":[60,65]}

% Apply the SNRMatrix to each robot 
SNRMatrix = SNRMatrix(1:numel(R)-1,1:numel(R)-1);
temp = SNRlow * (ones(numel(R),numel(R)) - diag(ones(1,numel(R)))); % set all options to low
temp(2:end,1:end-1) = SNRMatrix * (SNRhigh-SNRlow) + temp(2:end,1:end-1); % update bottom left to high where true
temp(1:end-1,2:end) = SNRMatrix' * (SNRhigh-SNRlow) + temp(1:end-1,2:end); % update top right to high where true
temp = temp .* reshape([R.comDevices],numel(R),[]); % only keep SNR at active connections
temp = mat2cell(temp,[ones(1,numel(R))]); % make lists for each robot
[R.SNR] = temp{:}; clearvars temp SNRhigh SNRlow SNRMatrix;

R = makeTrajectory(R,num_points);


% Predefine odom and CSI data timetables
odomT = timetable('Size',[0 7], 'RowTimes',datetime.empty(0,1), ...
    'VariableTypes',{'double','double','double','double','double','double','double'}, ...
    'VariableNames', {'pos_x','pos_y','pos_z','quat_x','quat_y','quat_z','quat_w'});
csiT = timetable('Size',[0 3], 'RowTimes',datetime.empty(0,1), ...
    'VariableTypes',{'double','string','cell'}, ...
    'VariableNames',{'itr','MAC','CSI'});

% Store the predefined and overwrite previous data
[R.odomT] = deal(odomT);
[R.csiT] = deal(csiT);
clearvars odomT csiT
%[text] ## Run Experiment


% Reset robots and time
call(resetSimulation);

% Do the run
R = runExperiment(R,rate,centerFrequency,bandwidth,numSubC);

% Determing average Hz from the iteration
runTime = seconds(R(1).odomT.Time(end) - R(1).odomT.Time(1)); % duration in sec
fprintf("average Hz is %.2f\n", (numel(R(1).odomT))/runTime) %[output:2f5a714c]
%%
%[text] ## Visualize and Analyze Results (Bartlett Estimation)
transmitters = [R.trajType]=="static";
receivers = ~transmitters;
AoA_est_all=nan(numel(R));
AoA_true_all=nan(numel(R));
addStartPos2Odom = 0; % add to true when odom starts at 0,0 rather than spawn pos

% True positions
spawnPos = reshape([R.posStart],4,[])'; %spawn positions
trajStartPos = reshape(cell2mat(arrayfun(@(x) x.odomT{1, ["pos_x","pos_y","pos_z"]}, R, 'UniformOutput', false)),3,numel(R))'; % first entries from odometry
allPos = trajStartPos + (spawnPos(:,1:3) * addStartPos2Odom);

% Enable/disable plotting
plotOdom =   true; %[control:checkbox:9994]{"position":[14,18]}
plotSignal = true; %[control:checkbox:5641]{"position":[14,18]}
plotAoA =    true; %[control:checkbox:10f0]{"position":[14,18]}
plotSave =   true; %[control:checkbox:1e2b]{"position":[14,18]}
plotVisible = "off"; % display in editor %[control:dropdown:5ceb]{"position":[15,20]}

% Get range for plotting odom
odomPlotRange = get_max_range(R);
% Predefine figure storage for saving files
figsOdom = cell(1,numel(R));
figsSignal = cell(numel(R),numel(R));
figsAoA = cell(numel(R),numel(R));


% Loop over receivers
for i=find(receivers)

    % Plot odometry
    if plotOdom
        figsOdom{i} = odomFigure(R(i).odomT,odomPlotRange,i,plotVisible);
    end

    % Loop over transmitters
    for j=find(R(i).comDevices)
        movingWindow = 1*num_points; %rendezvous
        
        % Process CSI raw data (interpolate, cancel CFO)
        [CSI_ij,figSignal] = processCSI(R,i,j,movingWindow,plotSignal,plotVisible);
        figsSignal{i,j} = figSignal; % store figure handle
        
        %% AoA estimation
        nbeta =          360; % azimuth resolution %[control:editfield:6f21]{"position":[26,29]}
        ngamma =         180; % elevation resolution %[control:editfield:0560]{"position":[26,29]}
        beta_min = -180; % minimum azimuth %[control:slider:00e0]{"position":[20,24]}
        beta_max = 180; % maximum azimuth %[control:slider:7978]{"position":[20,23]}
        gamma_min = 0; % minimum elevation %[control:slider:04f1]{"position":[21,22]}
        gamma_max = 180; % maximum elevation %[control:slider:6864]{"position":[21,24]}
        betaList = linspace(deg2rad(beta_min), deg2rad(beta_max), nbeta).';
        gammaList = linspace(deg2rad(gamma_min), deg2rad(gamma_max), ngamma);
        
        % Estimate AoA
        [~, AOA_angle, figAoA] = bartlett_wrap(CSI_ij,betaList,gammaList, lambda, i, j, plotAoA,plotVisible);
        figsAoA{i,j} = figAoA; % store figure handle
        AoA_est_all(i,j) = wrapTo180(AOA_angle+180); % flip to get angle from transmitter to receiver
        
        % Calculate true AoA
        rPos = allPos(i,1:2);
        tPos = allPos(j,1:2);
        AoA_true_all(i,j) = calculate_bearing(tPos, rPos);
    end
end
AoA_error_all = wrapTo180(AoA_est_all - AoA_true_all) %[output:6506cd44]

% reduce to only receiver-transmitter pairs
AoA_est = AoA_est_all(receivers,transmitters);
AoA_true = AoA_true_all(receivers,transmitters);
AoA_error = wrapTo180(AoA_est - AoA_true);

% Call function
[p_error, p_est, p_true, R, figSetup, figLocalize] = localizeAndPlot(R, allPos, receivers, transmitters, AoA_est_all, plotVisible);
p_error %[output:8b30acd6]

if plotSave %[output:group:7ad9df90]
    folder_base = "C:\Users\b.dijkstra\OneDrive - University of Groningen\Documenten\Industrial Engineering & Management\2.0 MSc Industrial Engineering & Management\4.2 Research Project - WiFi-CSI-Sensing\_ICRA25\MATLAB\MRS-Results"; %[control:filebrowser:79b9]{"position":[19,233]}
    folder_run = sprintf("\\%s\\",datetime("now","Format","dd-MM-uuuu_HH-mm-ss"));
    folder = strcat(folder_base, folder_run);
    mkdir(folder); % Create the directory for saving results

    % Save figures to the created directory
    saveas(figSetup, fullfile(folder, 'setup_figure.png'));
    saveas(figLocalize, fullfile(folder, 'localize_figure.png'));
    save(fullfile(folder, 'results.mat'),"R","p_true","p_est","p_error","AoA_est","AoA_true","AoA_error",'-mat') %[output:2a00f09e] %[output:1d765fac] %[output:45780115] %[output:087a9b58] %[output:24cdb014] %[output:7f65dd63] %[output:5bd43b8f] %[output:50e3bec0] %[output:49639d24] %[output:9a23175a] %[output:154a342c] %[output:74560cc9] %[output:79566479] %[output:84855855] %[output:052e1cbe] %[output:16536903] %[output:271bf3e1] %[output:72ab4671] %[output:922ebbb3] %[output:3515b8cc] %[output:9445f046] %[output:7618ea7b] %[output:588d1f11] %[output:9b272d43]
    for i = find(receivers)
        if ~isempty(figsOdom{i})
            saveas(figsOdom{i}, fullfile(folder, sprintf('odom_figure_%d.png', i)));
        end
    end
    for i = find(receivers)
        for j = find(R(i).comDevices)
            if ~isempty(figsSignal{i,j})
                saveas(figsSignal{i,j}, fullfile(folder, sprintf('signal_figure_%d_%d.png', i, j)));
            end
        end
    end
    for i = find(receivers)
        for j = find(R(i).comDevices)
            if ~isempty(figsAoA{i,j})
                saveas(figsAoA{i,j}, fullfile(folder, sprintf('AoA_figure_%d_%d.png', i, j)));
            end
        end
    end
end %[output:group:7ad9df90]
%%
%[text] ## Repeated experiments
% Monte carlo settings
mc_itrs = 100;
f = waitbar(0,sprintf("start %i iterations",mc_itrs)); %[output:5709c9db]

% initial variables
transmitters = [R.trajType]=="static";
receivers = ~transmitters;
addStartPos2Odom = 0; % add to true when odom starts at 0,0 rather than spawn pos
spawnPos = reshape([R.posStart],4,[])'; %spawn positions


% Enable/disable plotting
plotOdom =   false;
plotSignal = false;
plotAoA =    false;
plotSave =   false;
plotVisible =false; % display in editor

%% AoA estimation
nbeta =          360; % azimuth resolution %[control:editfield:1bb9]{"position":[18,21]}
ngamma =         180; % elevation resolution %[control:editfield:64f4]{"position":[18,21]}
beta_min = -180; % minimum azimuth %[control:slider:4267]{"position":[12,16]}
beta_max = 180; % maximum azimuth %[control:slider:4ad9]{"position":[12,15]}
gamma_min = 0; % minimum elevation %[control:slider:3646]{"position":[13,14]}
gamma_max = 180; % maximum elevation %[control:slider:66a3]{"position":[13,16]}
betaList = linspace(deg2rad(beta_min), deg2rad(beta_max), nbeta).';
gammaList = linspace(deg2rad(gamma_min), deg2rad(gamma_max), ngamma);

% pre define lists results
mc_p_error_4=nan(mc_itrs,1);
mc_p_error_5=nan(mc_itrs,1);
mc_p_est_4=nan(mc_itrs,2);
mc_p_est_5=nan(mc_itrs,2);
mc_AoA_error_4=nan(mc_itrs,numel(R));
mc_AoA_error_5=nan(mc_itrs,numel(R));
odomT = timetable('Size',[0 7], 'RowTimes',datetime.empty(0,1), ...
    'VariableTypes',{'double','double','double','double','double','double','double'}, ...
    'VariableNames', {'pos_x','pos_y','pos_z','quat_x','quat_y','quat_z','quat_w'});
csiT = timetable('Size',[0 3], 'RowTimes',datetime.empty(0,1), ...
    'VariableTypes',{'double','string','cell'}, ...
    'VariableNames',{'itr','MAC','CSI'});
%[text] ## Monte carlo simulations
for mc=1:mc_itrs %[output:group:3d6035c4]

    % waitbar
    waitbar(mc/mc_itrs,f,sprintf("itr %i/%i",mc,mc_itrs)); %[output:5709c9db]

    % Reset data
    [R.odomT] = deal(odomT);
    [R.csiT] = deal(csiT);

    % Reset robots and time
    call(resetSimulation);
    
    % Do the run
    R = runExperiment(R,rate,centerFrequency,bandwidth,numSubC); %[output:0d422864]
    
    % Determing average Hz from the iteration
    runTime = seconds(R(1).odomT.Time(end) - R(1).odomT.Time(1)); % duration in sec
    fprintf("run %i, average Hz is %.2f\n", mc, (numel(R(1).odomT))/runTime) %[output:7f10862f]
    
    trajStartPos = reshape(cell2mat(arrayfun(@(x) x.odomT{1, ["pos_x","pos_y","pos_z"]}, R, 'UniformOutput', false)),3,numel(R))'; % first entries from odometry
    allPos = trajStartPos + (spawnPos(:,1:3) * addStartPos2Odom);
    AoA_est_all=nan(numel(R));
    AoA_true_all=nan(numel(R));
    
    % Predefine figure storage for saving files
    figsSignal = cell(numel(R),numel(R));
    figsAoA = cell(numel(R),numel(R));
    
    
    % Loop over receivers
    for i=find(receivers)
    
        % Loop over transmitters
        for j=find(R(i).comDevices)
            movingWindow = 1*num_points; %rendezvous
            
            % Process CSI raw data (interpolate, cancel CFO)
            [CSI_ij,~] = processCSI(R,i,j,movingWindow,plotSignal,plotVisible);
            
            % Estimate AoA
            [~, AOA_angle, ~] = bartlett_wrap(CSI_ij,betaList,gammaList, lambda, i, j, plotAoA,plotVisible);
            AoA_est_all(i,j) = wrapTo180(AOA_angle+180); % flip to get angle from transmitter to receiver
            
            % Calculate true AoA
            rPos = allPos(i,1:2);
            tPos = allPos(j,1:2);
            AoA_true_all(i,j) = calculate_bearing(tPos, rPos);
        end
    end
    AoA_error_all = wrapTo180(AoA_est_all - AoA_true_all);

    % Call function
    [p_error, p_est, ~, ~, ~, ~] = localizeAndPlot(R, allPos, receivers, transmitters, AoA_est_all, plotVisible);
    
    % hardcode the values in some lists
    mc_AoA_error_4(mc,:)=AoA_error_all(4,:);
    mc_p_error_4(mc)=p_error(1);
    mc_p_est_4(mc,:)=p_est(1,:);
    if numel(R)>4
        mc_AoA_error_5(mc,:)=AoA_error_all(5,:);
        mc_p_error_5(mc)=p_error(2);
        mc_p_est_5(mc,:)=p_est(2,:);
    end
end %[output:group:3d6035c4]
% waitbar
close(f)
%%


%mc_AoA_error_4 = rmmissing(mc_AoA_error_4);
mc_p_error_4 = rmmissing(mc_p_error_4);
mc_p_est_4 = rmmissing(mc_p_est_4);

mean(mc_p_error_4) %[output:93ac956b]
mean(mc_p_error_5) %[output:0e6cbb9e]
median(mc_p_error_4) %[output:5002d5ef]
median(mc_p_error_5) %[output:0b87372c]
std(mc_p_error_4) %[output:526f36da]
std(mc_p_error_5) %[output:55315153]

mean(abs(mc_AoA_error_4),1) %[output:04a2a651]
mean(abs(mc_AoA_error_5),1) %[output:3ee7dd84]
median(abs(mc_AoA_error_4),1) %[output:534dc215]
median(abs(mc_AoA_error_5),1) %[output:86113513]
std(abs(mc_AoA_error_4),1) %[output:4090efa9]
std(abs(mc_AoA_error_5),1) %[output:48f83406]
%%
function [max_range] = get_max_range(R)
% GET_MAX_RANGE Compute maximum position range across all robots
%   [max_range_x, max_range_y] = get_max_range(R) calculates the maximum
%   position range for all robots in struct array R with field odomT

    max_range = 0;

    for i = 1:numel(R)
        if isfield(R(i), 'odomT') && ismember('pos_x', R(i).odomT.Properties.VariableNames)
            x_data = R(i).odomT.pos_x;
            y_data = R(i).odomT.pos_y;
            
            range_x = max(x_data) - min(x_data);
            range_y = max(y_data) - min(y_data);
            
            if range_x > max_range
                max_range = range_x;
            end
            if range_y > max_range
                max_range = range_y;
            end
        end
    end
end


function hFig = odomFigure(odomT, range, i, plotVisible)
% ODOMFIGURE Plots the trajectory from a timetable with pos_x and pos_y columns.
%   odomFigure(odomT, range_x, range_y) plots the trajectory and centers the axis
%   limits so the trajectory is in the middle of a box with width range_x and height range_y.

    % Check input
    if ~istimetable(odomT)
        error('Input must be a timetable.');
    end
    if ~all(ismember({'pos_x','pos_y'}, odomT.Properties.VariableNames))
        error('Timetable must contain variables ''pos_x'' and ''pos_y''.');
    end
    if ~exist("plotVisible","var")
        plotVisible = 'on'; % default to show
    end

    % Extract data
    x = odomT{:,'pos_x'};
    y = odomT{:,'pos_y'};

    % Compute center
    mid_x = (min(x) + max(x)) / 2;
    mid_y = (min(y) + max(y)) / 2;

    % Compute axis limits
    xlim_centered = [mid_x - range/2, mid_x + range/2];
    ylim_centered = [mid_y - range/2, mid_y + range/2];

    % Plot
    hFig = figure('visible',plotVisible,"Theme","Light");
    plot(x, y, 'b-', 'LineWidth', 1.5);
    hold on;
    scatter(x(1), y(1), 60, 'g', 'filled', 'DisplayName', 'Start');
    scatter(x(end), y(end), 60, 'r', 'filled', 'DisplayName', 'End');
    hold off;
    grid on;
    axis equal;
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    title(sprintf('Robot %i Odometry Trajectory',i));
    xlim(xlim_centered);
    ylim(ylim_centered);
    legend('Trajectory', 'Start', 'End', 'Location', 'best');
    set(gca, 'FontSize', 12);
end



function [p_error_list, p_est_list, p_true_list, R, hFig1, hFig2] = localizeAndPlot(R, allPos, receivers, transmitters, AoA_est, plotVisible)
% LOCALIZEANDPLOT Perform AoA-based localization and visualization
% Inputs:
%   R            - Struct array with fields:
%                  .odomT (timetable: pos_x, pos_y)
%                  .comDevices (logical array robot communications)
%   allPos       - [Nx2] matrix of ground truth positions
%   receivers    - [1xN] logical array (true for mobile robots)
%   transmitters - [1xN] logical array (true for static robots)
%   AoA_est      - [numReceivers x numTransmitters] matrix of AoA estimates (degrees)
%
% Outputs:
%   p_est    - [numReceivers x 2] estimated positions
%   p_error  - [numReceivers x 1] localization errors

    if ~exist("plotVisible","var")
        plotVisible = 'on'; % default to show
    end
    
    checkAllCombinations = true;
    
    AoA_est_all=AoA_est;
    
    % Extract positions
    p_true_list = allPos(receivers, 1:2);
    p_transmitters = allPos(transmitters, 1:2);
    numR = nnz(receivers);  % Number of receivers
    numT = nnz(transmitters);  % Number of transmitters
    
    % Preallocate results
    %p_est = cells(numR, 3);
    
    % Localization via optimization
    for i = find(receivers)
        if checkAllCombinations
            % All combinations of 2+ transmitters
            combinationsList = generate_transmitter_combinations(R(i).comDevices); % cell array of logical lists indicating participating robots
            p_results = table('Size',[length(combinationsList),5], ...
                'VariableTypes',["double","string","double","double","double"], ...
                'VariableNames',["combID","combList","p_est_x","p_est_y","p_error"]);
            for j=1:length(combinationsList)
    
                theta = deg2rad(AoA_est_all(i, combinationsList{j}));
                n_vectors = [cos(theta); sin(theta)]';
    
                combList = num2str(combinationsList{j}); % string representation of the logical list
                combID = bin2dec(combList); % binary representation of the boolean array
                options = optimoptions('lsqnonlin', 'Display', 'off', 'Algorithm', 'levenberg-marquardt');
                p_est = lsqnonlin(@(p) dist_to_lines(p, allPos(combinationsList{j},1:2), n_vectors), ...
                                        mean(allPos(combinationsList{j},1:2)), [], [], options);
                p_error = vecnorm(p_est - allPos(i,1:2), 2, 2); 
                p_results(j,:) = {combID,combList,p_est(1),p_est(2),p_error};
            end
            p_est_list(sum(receivers(1:i)),:) = p_est; % store last entry with all connections as main source
            R(i).p_results=p_results;
        else
            theta = deg2rad(AoA_est_all(i, :));
            activeCom = ~isnan(theta);
            n_vectors = [cos(theta(activeCom)); sin(theta(activeCom))]';
            
            options = optimoptions('lsqnonlin', 'Display', 'off', 'Algorithm', 'levenberg-marquardt');
            p_est_list(i,:) = lsqnonlin(@(p) dist_to_lines(p, p_transmitters(activeCom,:), n_vectors), ...
                                    mean(p_transmitters(activeCom,:)), [], [], options);
        end
    end
    
    % Calculate errors
    p_error_list = vecnorm(p_est_list - p_true_list, 2, 2);
    

%% Plot 1 setup overview
    % Visualization
    hFig1 = figure("Visible",plotVisible,"Position",[909 320 560 350],"Theme","Light"); 
    hold on; grid on; axis equal;
    %title('Multi-Robot Localization using AoA');
    xlabel('X Position (m)'); 
    ylabel('Y Position (m)');
    

    
    % % 2. Plot AoA rays
    % ray_length = 7;  % Meters
    % for i = find(receivers)
    %     theta = deg2rad(AoA_est_all(i,R(i).comDevices)); % With reversed bearing (+180deg) (static->moving)
    %     dir_vec = [cos(theta(:)), sin(theta(:))];  % Direction calculation
    % 
    %     p_communicators = allPos(R(i).comDevices,1:2);
    % 
    %     % Draw rays
    %     quiver(p_communicators(:,1), p_communicators(:,2), ...
    %         dir_vec(:,1)*ray_length, dir_vec(:,2)*ray_length, 0, ...
    %         'LineWidth', 1, 'MaxHeadSize', 0.1, 'DisplayName', "AoA R"+i);
    % end
    
    % 3. Plot ground truth connections
    commAll = reshape([R.comDevices], numel(R), []);
    commRT = commAll(receivers, transmitters);
    [recIdx, transIdx] = find(commRT);
    x_lines = [p_transmitters(transIdx,1), p_true_list(recIdx,1)]';
    y_lines = [p_transmitters(transIdx,2), p_true_list(recIdx,2)]';
    plot(x_lines, y_lines, 'r:', 'LineWidth', 1, 'HandleVisibility', 'off');
    if numel(R)<=4 % hardcoded test with 4 of 6 experiments
        % Handle legend nicely (only when no other lines need to be drawn).
        plot(nan, nan, 'r:', 'LineWidth', 1, 'DisplayName', 'Ground Truth');
    else
        % manually draw line between two moding nodes(4 and 5).
        plot(allPos(4:5,1), allPos(4:5,2), 'r:', 'LineWidth', 1, 'DisplayName', 'Ground Truth');
    end

    % 1. Plot mobile robot trajectories
    for i = find(receivers)
        odom = R(i).odomT{:, ["pos_x","pos_y"]};
        plot(odom(:,1), odom(:,2), 'LineWidth', 1.5, ...
            "DisplayName", "Odom R" + i);
    end
    
    % 4. Plot static robots
    scatter(p_transmitters(:,1), p_transmitters(:,2), 30, 'filled', ...
            'DisplayName', 'Static Robots');
    transmitterNames = "Static " + string(find(transmitters));
    text(p_transmitters(:,1), p_transmitters(:,2)+0.2, transmitterNames, ...
         'VerticalAlignment','bottom', 'HorizontalAlignment','center', ...
         'FontSize',10,'FontWeight','bold');
    
    % 5. Plot estimated/true positions
    % scatter(p_est_list(:,1), p_est_list(:,2), 120, 'y*', 'LineWidth', 1, ...
    %         'DisplayName', 'Est Pos');
    % scatter(p_true_list(:,1), p_true_list(:,2), 150, 'pentagram', 'LineWidth', 1, ...
    %         'DisplayName', 'True Pos');
    receiverNames = "Mobile " + string(find(receivers));
    text(p_true_list(:,1), p_true_list(:,2)+0.2, receiverNames, ...
         'VerticalAlignment','bottom', 'HorizontalAlignment','center', ...
         'FontSize',10,'FontWeight','bold');
    
    % Set axis limits to best fit the nodes
    xlim_min = min(allPos(:,1));
    xlim_max = max(allPos(:,1));
    xlim_range = xlim_max - xlim_min;
    ylim_min = min(allPos(:,2));
    ylim_max = max(allPos(:,2));
    ylim_range = ylim_max - ylim_min;
    
    xlim_min = xlim_min - xlim_range * 0.1;
    xlim_max = xlim_max + xlim_range * 0.1;
    ylim_min = ylim_min - ylim_range * 0.2;
    ylim_max = ylim_max + ylim_range * 0.2;
    
    xlim([xlim_min, xlim_max])
    ylim([ylim_min, ylim_max])
    
    % Finalize plot
    legend('Location', 'southoutside','NumColumns',2);
    set(gca, 'FontSize', 12);

%% Plot 2 localisation performance
    % Visualization
    hFig2 = figure("Visible",plotVisible,"Position",[909 320 560 350],"Theme","Light"); 
    hold on; grid on; axis equal;
    %title('Multi-Robot Localization using AoA');
    xlabel('X Position (m)'); 
    ylabel('Y Position (m)');
    
    % % 1. Plot mobile robot trajectories
    % for i = find(receivers)
    %     odom = R(i).odomT{:, ["pos_x","pos_y"]};
    %     plot(odom(:,1), odom(:,2), 'LineWidth', 1.5, ...
    %         "DisplayName", "Odom R" + i);
    % end
    
    % 2. Plot AoA rays
    ray_length = 7;  % Meters
    for i = find(receivers)
        theta = deg2rad(AoA_est_all(i,R(i).comDevices)); % With reversed bearing (+180deg) (static->moving)
        dir_vec = [cos(theta(:)), sin(theta(:))];  % Direction calculation
        
        p_communicators = allPos(R(i).comDevices,1:2);
    
        % Draw rays
        quiver(p_communicators(:,1), p_communicators(:,2), ...
            dir_vec(:,1)*ray_length, dir_vec(:,2)*ray_length, 0, ...
            'LineWidth', 1, 'MaxHeadSize', 0.1, 'DisplayName', "AoA R"+i);
    end
    
    % 3. Plot ground truth connections
    commAll = reshape([R.comDevices], numel(R), []);
    commRT = commAll(receivers, transmitters);
    [recIdx, transIdx] = find(commRT);
    x_lines = [p_transmitters(transIdx,1), p_true_list(recIdx,1)]';
    y_lines = [p_transmitters(transIdx,2), p_true_list(recIdx,2)]';
    plot(x_lines, y_lines, 'r:', 'LineWidth', 1, 'HandleVisibility', 'off');
    plot(nan, nan, 'r:', 'LineWidth', 1, 'DisplayName', 'Ground Truth');
    
    % 4. Plot static robots
    scatter(p_transmitters(:,1), p_transmitters(:,2), 30, 'filled', ...
            'DisplayName', 'Static Robots');
    transmitterNames = "Static " + string(find(transmitters));
    text(p_transmitters(:,1), p_transmitters(:,2)+0.2, transmitterNames, ...
         'VerticalAlignment','bottom', 'HorizontalAlignment','center', ...
         'FontSize',12,'FontWeight','bold');
    
    % 5. Plot estimated/true positions
    scatter(p_est_list(:,1), p_est_list(:,2), 30, '*', 'LineWidth', 1, ...
            'DisplayName', 'Est Pos');
    scatter(p_true_list(:,1), p_true_list(:,2), 30, 'pentagram', 'LineWidth', 1, ...
            'DisplayName', 'True Pos');
    receiverNames = "Mobile " + string(find(receivers));
    text(p_true_list(:,1), p_true_list(:,2)+0.2, receiverNames, ...
         'VerticalAlignment','bottom', 'HorizontalAlignment','center', ...
         'FontSize',12,'FontWeight','bold');
    
    % Set axis limits to best fit the nodes
    xlim_min = min(allPos(:,1));
    xlim_max = max(allPos(:,1));
    xlim_range = xlim_max - xlim_min;
    ylim_min = min(allPos(:,2));
    ylim_max = max(allPos(:,2));
    ylim_range = ylim_max - ylim_min;
    
    xlim_min = xlim_min - xlim_range * 0.1;
    xlim_max = xlim_max + xlim_range * 0.1;
    ylim_min = ylim_min - ylim_range * 0.2;
    ylim_max = ylim_max + ylim_range * 0.2;
    
    xlim([xlim_min, xlim_max])
    ylim([ylim_min, ylim_max])
    
    % Finalize plot
    legend('Location', 'southoutside','NumColumns',2);
    set(gca, 'FontSize', 12);
end

% Helper function (local)
function F = dist_to_lines(p, a, n)
    F = zeros(size(a,1),1);
    for i = 1:size(a,1)
        diff = p - a(i,:);
        proj = dot(diff, n(i,:)) * n(i,:);
        F(i) = norm(diff - proj);
    end
end

function boolCombinations = generate_transmitter_combinations(transmitters)
    activeIdx = find(transmitters);
    nActive = numel(activeIdx);
    boolCombinations = {};
    
    % Pre-calculate total combinations
    totalCombs = 0;
    for k = 2:nActive
        totalCombs = totalCombs + nchoosek(nActive, k);
    end
    
    % Preallocate cell array
    boolCombinations = cell(1, totalCombs);
    counter = 1;
    
    for k = 2:nActive
        comb = nchoosek(activeIdx, k);
        for i = 1:size(comb, 1)
            newArray = false(size(transmitters));
            newArray(comb(i,:)) = true;
            boolCombinations{counter} = newArray;
            counter = counter + 1;
        end
    end
end


%[text] 
%%
%[text] Helper function esperiment run and trajectory
function bearing_deg = calculate_bearing(tPos, rPos)
    % Calculate vector differences (ENU coordinates)
    delta_x = rPos(1) - tPos(1);  % East component
    delta_y = rPos(2) - tPos(2);  % North component
    
    % Compute bearing angle in radians
    bearing_rad = atan2(delta_y, delta_x);  % Mathematical angle from East
    
    % Convert to degrees and normalize to 0-360°
    bearing_deg = mod(rad2deg(bearing_rad), 360);
end


function R = runExperiment(R,rate,centerFrequency,bandwidth,numSubC)

% Toggle debug printing
debug = false;

% Get info on ros simulation timing
rosparameters.sim_time = rosparam("get", "/use_sim_time");
rosparameters.time_step = rosparam("get", "/gazebo/time_step");

startTime = datetime('now');

    %Activate drones
    for i = 1:numel(R)
        if ~R(i).is3D % all nodes that are drones
            continue;
        end
        % Set to OFFBOARD mode
        modeReq = rosmessage(R(i).cltSetMode);
        modeReq.CustomMode = 'OFFBOARD';
        call(R(i).cltSetMode, modeReq);
        
        % Arm drone
        armReq = rosmessage(R(i).cltArming);
        armReq.Value = true;
        call(R(i).cltArming, armReq);
    end
    
    % Drone initial position
    for t = 1:10
        for i = 1:numel(R)
            if ~R(i).is3D % all nodes that are drones
                continue;
            end
            R(i).msgPub.Pose.Position.X = 0;
            R(i).msgPub.Pose.Position.Y = 0; 
            R(i).msgPub.Pose.Position.Z = 2; % Takeoff altitude
            send(R(i).rosPub, R(i).msgPub);
            waitfor(rate);
        end
    end
    
    
    % loop over time
    for t = 1:numel(R(1).vx)
    
        %%% Subscribe Odom
        % Save position and orientation for all actively communicating nodes
        for i = nonzeros(any(reshape([R.comDevices],numel(R),[])).*(1:numel(R)))'
            %% Subscribe
            try
                msgSub = R(i).rosSub.receive(1);
            catch
                msgSub = R(i).rosSub.receive(1);
            end
            
            % Get odom from the stucts
            pos = struct2cell(msgSub.Pose.Pose.Position)';
            ori = struct2cell(msgSub.Pose.Pose.Orientation)';
    
            % Check ROS odom data
            if any(isnan(cell2mat([pos,ori])))
                warning("NaN value appeared on iteration:" + i);
                continue; % skip this iteration
            end
            
            % Store odom at node in timetable
            %curTime = datetime('now'); % local time
            simTime = double(msgSub.Header.Stamp.Sec)+double(msgSub.Header.Stamp.Nsec)*1e-9; % simulation time
            curTime = startTime + seconds(simTime);
            R(i).odomT(curTime,:) = [pos(2:end), ori(2:end)];
        end
        
        %%% Calculate CSI for all Active Signal Communications
        % loop over all actively communicating nodes
        for i = nonzeros(any(reshape([R.comDevices],numel(R),[])).*(1:numel(R)))'
            % Loop over all its active connections
            for j = nonzeros([R(i).comDevices] .* (1:numel(R)))'
                % Printing debug
                if debug
                    fprintf("Time %i, CSI %i to %i\n",t,i,j)
                end
                
                % Define input for CSI calc
                posRX = R(i).odomT{end, {'pos_x', 'pos_y', 'pos_z'}} + R(i).posStart(1:3); % add start pos to odom
                posTX = R(j).odomT{end, {'pos_x', 'pos_y', 'pos_z'}} + R(j).posStart(1:3);
                meanTime = mean([R(i).odomT.Time(end), R(j).odomT.Time(end)]); % average time between two odom measurement in seconds since the UNIX epoch
                deltaFrequency = R(i).deltaFrequency - R(j).deltaFrequency; % combined antenna offset
                SNR = mean([R(i).SNR(j), R(j).SNR(i)]); % average SNR configured between the two nodes for this connection
                
                % Calculate CSI
                CSI = csiCalc(posRX, posTX, posixtime(meanTime),...
                    centerFrequency,bandwidth,numSubC,deltaFrequency,SNR);
                
                % Store data at each robot
    	        R(i).csiT(meanTime,:) = {  % store at different time than odom
                    t, ...                        % itr counter
                    R(j).MAC, ...                 % MAC address of other node
                    CSI(:) ...                    % CSI values array and make sure its vertical array
                };
            end
        end
        
    
        %%% Live bearing calc for rendezvous
        if any(strcmp([R.trajType],"rendezvous"))
            
            % Rendezvous calculation interval settings
            rdvInterval = 100; % Update angle every X itrs
            rdvWindow = 500; % moving window of last X packets
    
            if mod(t,rdvInterval)==0
    
                % Freeze Gazebo and sim time for heavy calculations
                %pauseRequest = rosmessage(pauseClient);
                %call(pauseClient, pauseRequest);
                
                % Loop over all robots doing rendezvous
                for i = nonzeros(strcmp([R.trajType],"rendezvous").*(1:numel(R)))'
    
                    % Rendezvous target robot
                    j = R(i).trajRDVTarget;
                    
                    %%%% BARTLETT ESTIMATION FUNC HERE
        
                end
                %% update trajectory of the robot
    
            end
        end
                
    
    
    
        %%% Publish Trajectory
        % Update speeds for all actively communicating nodes
        for i = nonzeros(any(reshape([R.comDevices],numel(R),[])).*(1:numel(R)))'
            R(i).msgPub.Linear.X = R(i).vx(t);
            R(i).msgPub.Linear.Y = R(i).vy(t);
            R(i).msgPub.Angular.Z = R(i).rz(t);
            if R(i).is3D
                % Only update Z for drones
                R(i).msgPub.Linear.Z = R(i).vz(t);
            end
            send(R(i).rosPub,R(i).msgPub);
        end
    
        % Make sure to not overload the system
        waitfor(rate);
    end % end time loop
end


function R = makeTrajectory(R,num_points)

    for i=1:numel(R)
        switch R(i).trajType
            case "line"
                % Line
                azimuthDir = 0; % azimuth direction in deg 0-360
                trajMagnitude = R(i).trajVel;
                start_pos = [0;0;0]; %x,y
                end_pos = [cos(azimuthDir)*trajMagnitude; sin(azimuthDir)*trajMagnitude; 0]; %x,y
                waypoints = [start_pos, end_pos];
                [~, refVel, ~] = trapveltraj(waypoints, num_points,'PeakVelocity',0.1);
    
                % Store the velocities
                R(i).vx=refVel(1,:)/2; R(i).vy=refVel(2,:)/2; R(i).vz=refVel(3,:); % slow down by /2
            case "circle"
                % Circle
                if R(i).trajOmni
                    % Omnidirectional
                    radius = R(i).trajRad;
                    theta = linspace(0, 2*pi, num_points);
                    refPos = [radius*cos(theta); radius*sin(theta); zeros(1,num_points)];
                    
                    omega = 2*radius*2*pi/num_points;  % angular velocity per step
                    refVel = [-radius*sin(theta)*omega; radius*cos(theta)*omega; zeros(1,num_points)];
                    
                    % Store the velocities
                    R(i).vx=refVel(1,:); R(i).vy=refVel(2,:); R(i).vz=refVel(3,:); R(i).rz(t)=refVel(1,:)*0;
                else
                    % Differential
                    
                    R(i).vx = ones(1,num_points) * R(i).trajVel;
                    R(i).vy = zeros(1,num_points); R(i).vz = zeros(1,num_points);
                    R(i).rz = ones(1,num_points) * R(i).trajRot;
                end
    
            case "static"
                blank = zeros(1,num_points);
    
                % Store the velocities
                R(i).vx=blank; R(i).vy=blank; R(i).vz=blank; R(i).rz=blank;
                clearvars blank;
            case "random-walk"
                % 0-50% straight, 0-50% rotation, remaining % staight
                % make sure any part has at least 1.
                % assume not omnidirectional
                p1 = ceil(rand() * num_points/2);
                p2 = min(ceil(rand() * num_points/2) + p1,199);
                p3 = num_points;
                
                % Part 1
                vx(1:p1) = R(i).trajVel;
                vy(1:p1) = 0;
                vz(1:p1) = 0;
                rz(1:p1) = 0;

                % Part 2
                vx(p1+1:p2) = 0;
                vy(p1+1:p2) = 0;
                vz(p1+1:p2) = 0;
                rz(p1+1:p2) = R(i).trajRot;

                % Part 3
                vx(p2+1:p3) = R(i).trajVel;
                vy(p2+1:p3) = 0;
                vz(p2+1:p3) = 0;
                rz(p2+1:p3) = 0;

                % Store
                R(i).vx=vx; R(i).vy=vy; R(i).vz=vz; R(i).rz=rz;
            otherwise
                error("unknown traj type")
        end
    end

end
%[text] 
%[text] Helper functions for AoA estimation
function [CSI_ij, figSignal] = processCSI(R,i,j,movingWindow,plotSignal,plotVisible)

    if ~exist("plotVisible","var")
        plotVisible = 'on'; % default to show
    end

    %% Process CSI data to filter on node, moving window and matching pairs
    CSI_i = R(i).csiT(strcmp(R(i).csiT.MAC,R(j).MAC), :);                   % Filter CSI data on other node's MAC
    CSI_i = CSI_i(max(1, height(CSI_i) - movingWindow):end,["itr","CSI"]);  % CSI data within moving window
    CSI_i = renamevars(CSI_i,"CSI","CSI_i");                                % Rename for merging
    
    CSI_j = R(j).csiT(strcmp(R(j).csiT.MAC,R(i).MAC), :);                   % Filter CSI data on other node's MAC
    CSI_j = CSI_j(max(1, height(CSI_j) - movingWindow):end,["itr","CSI"]);  % CSI data within moving window 
    CSI_j = renamevars(CSI_j,"CSI","CSI_j");                                % Rename for merging
    
    CSI_j = timetable2table(CSI_j); % Only left join table can remain as timetable and keep datetime info
    
    % Inner join to only keep matching pairs and keep datetime of node i
    CSI_ij = innerjoin(CSI_i,CSI_j,"Keys",["itr","itr"]);
    
    %% Process odom data to match CSI
    odom_i = retime(R(i).odomT,CSI_ij.Time,'linear');                        % interpolated odom to CSI timestamps
    odom_i = renamevars(odom_i,["pos_x","pos_y","pos_z","quat_x","quat_y","quat_z","quat_w"], ...
        ["pos_xi","pos_yi","pos_zi","quat_xi","quat_yi","quat_zi","quat_wi"]); % rename for merging
    CSI_ij = [CSI_ij,odom_i];                                                % merge with CSI data
    
    % Convert to Euler angles (ZYX order)
    eul = quat2eul([CSI_ij.quat_wi, CSI_ij.quat_xi, ...
        CSI_ij.quat_yi, CSI_ij.quat_zi], "ZYX");
    
    % Assign to CSI_ij
    CSI_ij.eul_zi = eul(:,1);
    CSI_ij.eul_yi = eul(:,2);
    CSI_ij.eul_xi = eul(:,3);
    
                   
    % Define interpolation function for a single CSI array
    interpolateCSI = @(csi) (...
        mean(abs(csi(floor(numel(csi)/2):ceil(numel(csi)/2)))) * ...                     % Average magnitude
        exp(1i * polyval(polyfit(1:numel(csi), unwrap(angle(csi)), 1), (numel(csi)+1)/2)) ... % Phase
    );
    
    % Process timetable in one step
    CSI_ij(:,["CSI_int_i","CSI_int_j"]) = rowfun(...
        @(csi_i, csi_j) deal(...
            interpolateCSI(csi_i), ...  % CSI_int_i
            interpolateCSI(csi_j) ...   % CSI_int_j
        ), ...
        CSI_ij, ...
        'InputVariables', {'CSI_i', 'CSI_j'}, ...
        'OutputVariableNames', {'CSI_int_i', 'CSI_int_j'}, ...
        'ExtractCellContents', true ... % Required if CSI_i/j are cell arrays
    );
    
    %%% Cancel CFO
    CSI_ij(:,"CSI_cfo") = rowfun(@(i,j) i*j, CSI_ij, ...  % Multiply CSI_int_i and CSI_int_j
        'InputVariables', {'CSI_int_i', 'CSI_int_j'}, ...
        'OutputVariableName', 'CSI_cfo');

    %%% Plot signal
    if plotSignal
        figSignal = figure("Visible",plotVisible,"Position",[909 320 560 200],"Theme","Light");
        % tiledlayout(2, 1); % Create a 2x1 tiled chart layout
        % 
        % % Amplitude / Magnitude plot in the first tile
        % ax1 = nexttile; % Get the axes for the first tile
        % csi_amplitude = abs(CSI_ij.CSI_cfo);
        % scatter(ax1, 1:length(csi_amplitude), csi_amplitude,".");
        % %xlabel(ax1, x_label);
        % ylabel(ax1, "Amplitude (-)");
        % %title(ax1, sprintf("Signal Amplitude Robot %i to %i",i,j));
        
        % Phase plot in the second tile
        ax2 = nexttile; % Get the axes for the second tile
        csi_phase = (angle(CSI_ij.CSI_cfo));
        scatter(ax2, 1:length(csi_phase), csi_phase,".");
        %xlabel(ax2, x_label);
        ylabel(ax2, "Phase (rad)");
        ylim([-pi,pi])
        set(gca,'ytick',(-pi:pi/2:pi)) % where to set the tick marks
        set(gca,'yticklabels',{'-\pi','-\pi/2','0','\pi/2','\pi'}) % give them user-defined labels
        %title(ax2, sprintf("Signal Phase Robot %i to %i",i,j));
    else
        figSignal = [];
    end
end



function [AOA_profile, AOA_angle, figAoA] = bartlett_wrap(CSI_ij, betaList, gammaList, lambda, i, j, plotAoA, plotVisible)
    if ~exist("plotVisible","var")
        plotVisible = 'on'; % default to show
    end

    yawList = atan2(CSI_ij.pos_yi-CSI_ij.pos_yi(1), ...
        CSI_ij.pos_xi-CSI_ij.pos_xi(1)); % azimuth angles of rx and rx's initial position
    %yawList(1) = deg2rad(90); % Start at 90 deg based on reference frame
             
    pitchList = CSI_ij.eul_yi - CSI_ij.eul_yi(1);      % elevation angles of rx and rx's initial position
    rhoList = sqrt((CSI_ij.pos_xi-CSI_ij.pos_xi(1)).^2 + ...
        (CSI_ij.pos_yi-CSI_ij.pos_yi(1)).^2 + ...
        (CSI_ij.pos_zi-CSI_ij.pos_zi(1)).^2);       % distance between rx and rx's initial position 
    
    simple=false;
    
    % Compute the AOA profile
    [AOA_profile, AOA_angle] = bartlett_AOA_estimator(CSI_ij.CSI_cfo, yawList, pitchList, ...
        rhoList, lambda, betaList, gammaList, simple);

    if plotAoA
        figAoA = figure("Visible",plotVisible,"Position",[909 320 560 200],"Theme","Light");
        
        % % Side view plot (tile 1)
        % ax1 = nexttile; % Get the axes for the first tile
        % surf(ax1, rad2deg(betaList), rad2deg(gammaList), AOA_profile.', 'EdgeColor', 'none');
        % set(gcf,'Renderer','Zbuffer');            
        % xlabel(ax1, 'Azimuth (degrees)');
        % ylabel(ax1, 'Elevation (degrees)');
        % zlabel(ax1, 'Spectrum (-)');
        % xlim(ax1, [-180,180])
        % ylim(ax1, [0,180])
        % zlim(ax1, [0 max(max(AOA_profile))]);
        % title(ax1, sprintf('AOA Profile Robot %i to %i (Side View)',i,j));
       
        
        % Top view plot (tile 2)
        ax2 = nexttile; % Get the axes for the second tile
        surf(ax2, rad2deg(betaList), rad2deg(gammaList), AOA_profile.', 'EdgeColor', 'none');
        set(gcf,'Renderer','Zbuffer');
        view(ax2, 2);
        xlabel(ax2, 'Azimuth (degrees)');
        ylabel(ax2, 'Elevation (degrees)');
        zlabel(ax2, 'Estimated spectrum (-)');
        xlim(ax2, [-180,180]);
        ylim(ax2, [0,180]);
        colorbar;
        %title(ax2, sprintf('AOA Profile R%i to R%i (Top View)',i,j));
    else
        figAoA = [];
    end
end




%[text] ## 
%[text] ## 
%[text] ## 
%[text] ## 
%%
%[text] ## 
%[text] ## Visualize and Analyze Results (Bartlett Estimation)
  %[control:button:059b]{"position":[1,2]}
% Settings variables
bartlett.plot_AoA    = true;  % boolean to show AoA profile plot %[control:checkbox:2114]{"position":[24,28]}
bartlett.plot_signal = true;  % boolean to show signal properties plot %[control:checkbox:04b4]{"position":[24,28]}
bartlett.plot_odom   = true;  % boolean to show odometry trajectory plot %[control:checkbox:50b1]{"position":[24,28]}
bartlett.plot_popup  = false;  % boolean to make plots pop-up from live script %[control:checkbox:3740]{"position":[24,29]}
bartlett.plot_save   = false;  % boolean to save all checked plots %[control:checkbox:3edf]{"position":[24,29]}
bartlett.plot_prefix = "recorded-test1"; % prefix to name the plots %[control:editfield:1a08]{"position":[24,40]}
bartlett.fixed_yaw   = true;  % boolean to set manual yaw calculation instead of odom %[control:checkbox:4697]{"position":[24,28]}
bartlett.simple      = true;  % boolean for 2D computation %[control:checkbox:4aab]{"position":[24,28]}
bartlett.lambda = lambda;
bartlett.trajectory = traj.method; % Specify type of trajectory driven for plots

% convert timetable datetime to usable format
% Current time in seconds since the UNIX epoch (00:00:00 UTC on 1 January 1970)
%currentTime = posixtime(datetime('now'));

AOA_angle{controlR} = bartlett_wrapper(csi_data_RX,csi_data_TX,odom_rx,bartlett);
% True angle (deg)
txPosition = [0,0]; % assume odometry of tx does not move
true_angle=rad2deg(atan2(txPosition(2)+posStart(1,2)-mean(odom_rx(:,4))-posStart(controlR+1,2), ... % T_y - R_y
txPosition(1)+posStart(1,1)-mean(odom_rx(:,3))-posStart(controlR+1,1)));                            % T_x - R_x

% Error in deg, wrapped to [-180, 180] range.
error_angle=wrapTo180(true_angle-AOA_angle{controlR});

% Display the results
fprintf("True angle: %f deg\nFound angle: %f deg\nError angle: %f deg\n",true_angle,AOA_angle{controlR},error_angle)
%%
%[text] ## Store CSV and DAT Files, and Send to Linux Desktop.
% Send parameters
sendfiles = true;              % send files to linux machine  %[control:checkbox:4303]{"position":[13,17]}
sendusr='alex'; % linux username %[control:editfield:45d5]{"position":[9,15]}
senddir='/home/alex/wsr_data'; % linux directory %[control:editfield:7efa]{"position":[9,30]}

switch dataFormat
    case "WSR"
        SaveFiles(csi_data_RX,csi_data_TX,odom_rx,sendfiles,sendusr,senddir,IP_Remote,traj)
    case "Nexmon"

    otherwise
        p_error("unkown dataformat to store file")
end
%[text] 
%%
%[text] 
%[text] 
%[text] 
%[text] 
%[text] ## 
%[text] ### 
%[text] 
%[text] 
%[text] 
%[text] 
%[text] 
%[text] 
%[text] 
%[text] 
%[text] 
%[text] 
%[text] 
%[text] 
%[text] 
%%
%[text] # Helper functions
%[text] ### Spawning Nexus Cars
function [rosSub,rosPub,msgPub,cltArming,cltSetMode]=rosSpawn(sdf_path_ground,sdf_path_drone,R,debug,conf)
    is3D = [R.is3D];
%[text] ####     0. Debugging ROS
    if debug.ros
        disp("__Services__"); rosservice list
        disp("__Nodes_____"); rosnode list
        disp("__Topics____"); rostopic list
        disp("__Gazebo____"); rosnode('info','/gazebo')
    end
%[text] 1. Check ssh environment for spawning drones \
%[text] This checks if the bashrc is also loaded for non-interactive ssh sessions and wether the paths are set correctly.
% Only check if drones are used
if any(is3D) 
    % Executed from Windows device
    if ~strcmp(conf.RemoteIP, conf.LocalIP)
        check_command = sprintf('ssh %s@%s "printenv | grep ROS"', conf.RemoteUser, conf.RemoteIP);
        [~, cmdout] = system(check_command);
    else % linux device itself
        check_command = sprintf('printenv | grep ROS');
        [~, cmdout] = system(check_command);
    end
    
    % Example: cmdout is the string output from your SSH command
    lines = regexp(cmdout, '\n', 'split');
    ros_env = struct();
    
    for i = 1:length(lines)
        line = strtrim(lines{i});
        % Ignore empty lines and usage text
        if isempty(line) || startsWith(line, 'usage:')
            continue;
        end
        % Some lines may contain multiple variables concatenated by \n
        sublines = regexp(line, '\n', 'split');
        for j = 1:length(sublines)
            subline = strtrim(sublines{j});
            eqIdx = strfind(subline, '=');
            if ~isempty(eqIdx)
                key = strtrim(subline(1:eqIdx(1)-1));
                value = strtrim(subline(eqIdx(1)+1:end));
                ros_env.(key) = value;
            end
        end
    end
    
    % Now check for required variables
    required_vars = {'ROS_MASTER_URI', 'ROS_PACKAGE_PATH', 'ROS_DISTRO', 'ROS_ROOT'};
    missing_vars = required_vars(~isfield(ros_env, required_vars));
    
    if isempty(missing_vars)
        disp('ROS environment is properly configured.');
    else
        warning('Missing ROS variables: %s', strjoin(missing_vars, ', '));
    end
end

%[text] ####     1. Load Robot SDF Model as Text
%[text]  [https://answers.ros.org/question/382549/spawn-gazebo-model-editor-model-with-ros-service/](https://answers.ros.org/question/382549/spawn-gazebo-model-editor-model-with-ros-service/)
    % if there are any ground robots specified, load the model.
    if any(~is3D)
        % Check SDF/URDF file extension
        % Find the position of the last dot
        dotLocations = strfind(sdf_path_ground,'.');
        
        if isempty(dotLocations)
            model_extension_ground = ""; 
            error("No dot found in: "+sdf_path_ground)
        else
            % Extract substring after the last dot
            model_extension_ground = lower(extractAfter(sdf_path_ground, dotLocations(end)));  % Works for both string and char array[1][3]
            % Check if the extension is 3 or 4 characters
            if strlength(model_extension_ground) < 3 || strlength(model_extension_ground) > 4
                model_extension_ground = ""; % Not a valid extension length
                error("sdf/urdf ground robot file extension not correct size")
            elseif ~strcmp(model_extension_ground,"sdf") && ~strcmp(model_extension_ground,"urdf")
                model_extension_ground = ""; % Not a valid extension length
                error("sdf/urdf ground robot file extension not correct format")
            end
        end

        % read sdf as textfile
        sdf_table_ground = table2array(readtable(sdf_path_ground, 'FileType', "text",'ReadVariableNames',false,'ReadRowNames',false,'NumHeaderLines',0));
        sdf_string_ground = "";
        
        % iterate through every line of the sdf
        for n_sdf = 1:length(sdf_table_ground)
            % attach line to string
            sdf_string_ground = strcat(sdf_string_ground, sdf_table_ground{n_sdf}, "\n");
        end
        disp(sdf_string_ground)
    end
    
    % if there are any aerial drones specified, load the model.
    if any(is3D)
        % read sdf as textfile
        sdf_table_drone = table2array(readtable(sdf_path_drone, 'FileType', "text",'ReadVariableNames',false,'ReadRowNames',false,'NumHeaderLines',0));
        sdf_string_drone = "";
        
        % iterate through every line of the sdf
        for n_sdf = 1:length(sdf_table_drone)
            % attach line to string
            sdf_string_drone = strcat(sdf_string_drone, sdf_table_drone{n_sdf}, "\n");
        end
        disp(sdf_string_drone)
    end
    
    % Make sure meshes files are in directory on remote PC and gazebo path is set.
    fprintf('\nRUN ON REMOTE PC\n$ export GAZEBO_MODEL_PATH="$GAZEBO_MODEL_PATH:$HOME/model_folder/"\n')
%[text] ####     2. Test Gazebo Connection
    % Test if gazebo node is reachable
    try [~]=rosnode('info','/gazebo'); catch; error("Gazebo not reachable"); end
%[text] ####     3. Spawn robots (Create ROS Spwn Service and Spawn Multiple RXs)
    % Initiate spawn service
    if strcmp(model_extension_ground,"sdf")
        track_spawn_client = rossvcclient("/gazebo/spawn_sdf_model",'IsPersistent',false); %,'DataFormat','struct',"Timeout",10
    elseif strcmp(model_extension_ground,"urdf")
        track_spawn_client = rossvcclient("/gazebo/spawn_urdf_model",'IsPersistent',false);
    else
        error("spawning ground robot extension not supported: "+model_extension_ground)
    end
    %track_spawn_client = rossvcclient("/gazebo/spawn_urdf_model",'IsPersistent',false); %,'DataFormat','struct',"Timeout",10
    track_spawn_service = rosmessage(track_spawn_client);
    
    % Services for spawning pedestals
    box_spawn_client = rossvcclient("/gazebo/spawn_urdf_model",'IsPersistent',false);
    box_spawn_service = rosmessage(box_spawn_client);
    
    % Pre-define RX cells
    rosSub = cell(1,numel(R));
    rosPub = cell(1,numel(R));
    msgPub = cell(1,numel(R));
    landPub  = cell(1,numel(R)); % drone only
    cltArming = cell(1,numel(R)); % drone only
    cltSetMode = cell(1,numel(R)); % drone only
    
    for i=1:numel(R)
        % ROBOTS
        if ~is3D(i)
            % Create pedestal supports
            if R(i).posStart(3)>0
                

                box_height = R(i).posStart(3);
                box_urdf = generate_box_urdf(box_height);
                
                box_spawn_service.ModelXml = box_urdf;
                box_spawn_service.ModelName = ['pedestal_robot', num2str(i)];
                box_spawn_service.RobotNamespace = ['/', box_spawn_service.ModelName];
                box_spawn_service.ReferenceFrame = 'world';
                box_spawn_service.InitialPose.Position.X = R(i).posStart(1);
                box_spawn_service.InitialPose.Position.Y = R(i).posStart(2);
                box_spawn_service.InitialPose.Position.Z = box_height / 2;  % Center at half-height
                box_response = call(box_spawn_client, box_spawn_service, 'Timeout', 20);

                % Check spawn success
                if ~box_response.Success
                    if strcmp(box_response.StatusMessage,'SpawnModel: Failure - entity already exists.')
                        warning(box_spawn_service.ModelName + " already exists, skipped");
                        continue;
                    else
                        showdetails(box_response)
                        error(box_spawn_service.ModelName + " not spawned");
                    end
                end
            end

            % Spawn model parameters
            track_spawn_service.ModelXml = sdf_string_ground;
            track_spawn_service.ModelName = strcat("robot", string(i)); % model name
            track_spawn_service.RobotNamespace = strcat("/", track_spawn_service.ModelName); % namespace of robot
            track_spawn_service.ReferenceFrame = 'world'; % referenceframe
            track_spawn_service.InitialPose.Position.X = R(i).posStart(1); % set spawn location X
            track_spawn_service.InitialPose.Position.Y = R(i).posStart(2); % set spawn location Y
            track_spawn_service.InitialPose.Position.Z = R(i).posStart(3); % set spawn location Z
            
            % Spawn the robot
            response = call(track_spawn_client,track_spawn_service, 'Timeout', 20); % call service
            
            % Check spawn success
            if ~response.Success
                if strcmp(response.StatusMessage,'SpawnModel: Failure - entity already exists.')
                    warning(track_spawn_service.ModelName + " already exists, skipped");
                    continue;
                else
                    showdetails(response)
                    error(track_spawn_service.ModelName + " not spawned");
                end
            end
            
            % Store subscriber and publisher
            rosSub{i} = rossubscriber(strcat(track_spawn_service.RobotNamespace, '/odom'),'nav_msgs/Odometry','BufferSize',3,'DataFormat','struct');
            rosPub{i} = rospublisher(strcat(track_spawn_service.RobotNamespace, '/cmd_vel'),'geometry_msgs/Twist','DataFormat','struct');
        else
            % DRONES UAVS
            %--->>>  roslaunch px4 dynamic_drone_spawn.launch   ns:=uav0   ID:=0   x:=0 y:=0 z:=0   vehicle:=iris   mavlink_udp_port:=14560   fcu_url:="udp://:14540@localhost:14580"   tgt_system:=1 mavlink_tcp_port:=4560
            %uav_count = sum(is3D(1:i)) - 1; % [0, 1, .. N] for drones, misaligns with i as PX4 wants subsequent numbers
            uav_count = i;
            mavlink_udp_port = 14560 + uav_count;
            fcu_url = sprintf('udp://:%i@localhost:%i',14540 + uav_count, 14580 + uav_count);
            tgt_system = i + uav_count;
            mavlink_tcp_port = 4560 + uav_count;
            if ~strcmp(conf.RemoteIP, conf.LocalIP) % Spawn from Windows
                command = sprintf(['start cmd /c ssh %s@%s ' ...
                    'roslaunch px4 dynamic_drone_spawn.launch ' ...
                    'ns:=uav%i ID:=%i x:=%i y:=%i z:=%i ' ...
                    'vehicle:=iris mavlink_udp_port:=%i ' ...
                    'fcu_url:=%s tgt_system:=%i mavlink_tcp_port:=%i'], ...
                    conf.RemoteUser, conf.RemoteIP, i, i, ...
                    R(i).posStart(1),R(i).posStart(2),R(i).posStart(3), ...
                    mavlink_udp_port, fcu_url, tgt_system, mavlink_tcp_port)
            else % Spawn from Linux itself
                command = sprintf(['roslaunch px4 dynamic_drone_spawn.launch ' ...
                    'ns:=uav%i ID:=%i x:=%i y:=%i z:=%i ' ...
                    'vehicle:=iris mavlink_udp_port:=%i ' ...
                    'fcu_url:=%s tgt_system:=%i'], ...
                    i, i, R(i).posStart(1),R(i).posStart(2),R(i).posStart(3), ...
                    mavlink_udp_port, fcu_url, tgt_system);
            end
            system(command, '-echo');
            
            
            % Check spawn success
            pause(3) % by default wait a little
            spawned = false;
            timeoutBool = false;
            timeoutTime = 20; % 20 sec
            droneNameSpace = sprintf('/uav%i/', i);
            tic
            while ~any([spawned, timeoutBool])
                rostopics = rostopic("list");
                if any(contains(rostopics, droneNameSpace))
                    spawned = true; % escape loop
                elseif toc>timeoutTime
                    timeoutBool = true; % escape by timeout
                else
                    pause(0.5); % stay in loop
                end
            end
            if ~spawned % robot did not spawn after timeout
                error("R%i not spawned after %is timeout",i,timeoutTime);
            end

            % Store subscriber and publisher
            rosPub{i} = rospublisher(strcat(droneNameSpace, 'mavros/setpoint_position/local'), 'geometry_msgs/PoseStamped', 'DataFormat', 'struct');
            rosSub{i} = rossubscriber(strcat(droneNameSpace, 'mavros/local_position/odom'),'nav_msgs/Odometry','BufferSize',3,'DataFormat','struct');
            %landPub{i} = rospublisher("/land_message","DataFormat","struct");
            cltArming{i} = rossvcclient(strcat(droneNameSpace, 'mavros/cmd/arming'), 'DataFormat', 'struct');
            cltSetMode{i} = rossvcclient(strcat(droneNameSpace, 'mavros/set_mode'), 'DataFormat', 'struct');
        end
        
        
    
        % Make final publish message per RX
        msgPub{i} = rosmessage(rosPub{i});
    
        % Pause for gazebo to update
        pause(0.1)
    end
%[text] ####     4. Debugging Nexus Car
    % Check on the nexus car information
    if debug.nexus
        if ~is3D(1)
            rostopic info /r1_ground/odom
            rostopic info /r1_ground/cmd_vel
        elseif is3D(1)
            rostopic info /r1_drone/odom
            rostopic info /r1_drone/cmd_vel
        else
            warning("no debugging for this robot type")
        end
        rosmsg info geometry_msgs/Twist
        rosmsg info nav_msgs/Odometry
    end
end
%[text] #### Robot Pedestal (Hieght support)
% Function to generate box URDF with variable height
function urdf_str = generate_box_urdf(height)
    urdf_str = sprintf(['<robot name="pedestal">', ...
        '<link name="base_link">', ...
        '<inertial><mass value="1000"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>', ...
        '<visual><geometry><box size="0.1 0.1 %f"/></geometry></visual>', ...
        '<collision><geometry><box size="0.1 0.1 %f"/></geometry></collision>', ...
        '</link>', ...
        '</robot>'], height, height);
end
%[text] #### ROS Initalization
function master = InitROS(conf)
%%% Ping both IPs to check network connection
    % Checks for Windows or Linux
    if ispc
        CR = evalc("!ping -n 1 " + conf.RemoteIP);
        CL = evalc("!ping -n 1 " + conf.LocalIP);
    elseif isunix
        CR = evalc("!ping -c 1 " + conf.RemoteIP);
        CL = evalc("!ping -c 1 " + conf.LocalIP); 
    end
    lossR = regexp(CR, '([0-9]*)%.*loss', 'tokens');
    lossL = regexp(CL, '([0-9]*)%.*loss', 'tokens');
    connectedR = ~isempty(lossR) && str2double(lossR{1}{1})==0;
    connectedL = ~isempty(lossL) && str2double(lossL{1}{1})==0;
    
    if ~(connectedR && connectedL)
        error("Local IP reachable: %s.\nRemote IP reachable: %s.",mat2str(connectedL),mat2str(connectedR))
    end


    if conf.RosCoreLocal 
    %%% ROS Core Local (on Laptop)
            master = ros.Core;
            setenv('ROS_MASTER_URI',strcat('http://',conf.LocalIP,':11311'));
            setenv('ROS_IP', conf.LocalIP);
            setenv('ROS_HOSTNAME', conf.LocalIP);
            try rosinit(conf.LocalIP,11311); catch e; warning(e.message); end
            fprintf("\nRUN ON REMOTE PC\n$ export ROS_IP=%s\n$ export ROS_HOSTNAME=%s\n$ export ROS_MASTER_URI=http://%s:11311\n",IP_Remote,IP_Remote,IP_Local)
    else
    %%% ROS Core Remote (on Desktop)
            master={};
            setenv('ROS_MASTER_URI',strcat('http://',conf.RemoteIP,':11311'));
            setenv('ROS_IP', conf.LocalIP);
            setenv('ROS_HOSTNAME', conf.LocalIP);
            try rosinit(conf.RemoteIP,11311); catch e; warning(e.message); end
            fprintf("\nRUN ON REMOTE PC\n$ export ROS_IP=%s\n$ export ROS_HOSTNAME=%s\n$ export ROS_MASTER_URI=http://%s:11311\n",conf.RemoteIP,conf.RemoteIP,conf.RemoteIP)
    
    end

    %%% Success message
    disp("$ rosrun gazebo_ros gazebo")
end
%[text] ### 
%[text] ### Stop ROS and Clear MATLAB
function KillROS()
    % Prompt for confirmation
    confirm = input('Are you sure you want to shutdown ROS, clear the workspace, and close all figures? [y/n]: ', 's');
    
    if strcmpi(confirm, 'y') || strcmpi(confirm, 'yes')
        evalin("base","rosshutdown");
        evalin("base","clear all");
        evalin("base","close all");
        evalin("base","clc");
        disp("Close Gazebo CTRL+C in terminal and kill all remaining gazebo processes.")
    else
        disp("KillROS cancelled")
    end
end
%[text] 
%[text] ### Save & Send Files
%[text] This section creates a timestamped local directory on the MATLAB machine for archiving. Moreover, it sends the created files in a fixed format over to the Desktop PC to easily parse it through the toolbox without needing to change the config.json file.
function SaveFiles(csi_data_RX,csi_data_TX,odom_rx,sendfiles,sendusr,senddir,IP_Remote,traj)
    % Make new folder based on timestamp
    newFolder="CSI_" + string(datetime('now','Format','yyyy-MM-dd_HHmm')) + ... % timestamp suffix
            "_" + traj.method + "_v" + string(traj.vel) + "_a" + string(traj.ang) + "_r" + string(traj.rad); % config
    mkdir(userpath,newFolder) % ~user/Documents/MATLAB
    localdir=strcat(userpath,"\",newFolder,"\"); % ~user/Documents/MATLAB/CSI_.../
    
    % Scale CSI Data
    scaler = 256;
    for i=1:length(csi_data_RX)
        csi_data_RX(i).csi = scaler * csi_data_RX(i).csi;
        csi_data_TX(i).csi = scaler * csi_data_TX(i).csi;
    end
    
    % Create TX data based on RX data
    %csi_data_tx=csi_data;
    %for i=1:length(csi_data_tx)
    %    csi_data_tx(i).mac = '0123456789CD';
    %end
    
    % Make the local files
    writematrix(odom_rx, strcat(localdir,"a_odom_rx.csv")) % Make CSV
    write_bf_file(strcat(localdir,"a_csi_rx.dat"), csi_data_RX) % Make DAT RX
    write_bf_file(strcat(localdir,"a_csi_tx.dat"), csi_data_TX) % Make DAT TX
    
    % Send complete folder to Desktop
    if sendfiles
        command = 'scp -r '+ strcat(localdir,"*") + ' ' + sendusr + '@' + IP_Remote + ':' + senddir;
        system('start cmd /c ' + command); % Only need to enter the password manually in terminal

        fprintf("ON DESKTOP PC\n$ cd ~/WSR_Project/WSR-Toolbox-cpp/wsr_build/\n$ ./test_wsr gt\n")
    end
end
%[text] 
%[text] ### Moving Robot to Position
function MoveNexus(MoveName,MovePos)
    % MOVING SERVICE DOES NOT UPDATE INTERNAL ODOMETRY ESTIMATION OF THE CAR
    
    % Moving client service
    clientMove = rossvcclient('/gazebo/set_model_state');
    msgMove = rosmessage(clientMove);
    
    % Move TX out of the way
    msgMove.ModelState.ModelName = MoveName;
    msgMove.ModelState.Pose.Position.X = MovePos(1);
    msgMove.ModelState.Pose.Position.Y = MovePos(2);
    msgMove.ModelState.Pose.Position.Z = MovePos(3);
    response = call(clientMove, msgMove);
    
    % Check move success
    if ~response.Success
        error("Nexus robot move unsuccessful")
    end
end
%[text] 
%[text] #### ROS RUN (old)
function [csi_data_RX,csi_data_TX,odom_rx]=rosRun(controlR, vx, vy, rate, centerFrequency, bandwidth, numSubC, deltaFrequency, SNR, ...
    posStart,rosSub_TX,rosSub_RX,rosPub_RX,msgPub_RX,dataFormat)
%%% Pre-define CSI, Odometry and Publish Message
    % trajectory size
    i_max=length(vx);

    % Fixed struct layouts
    switch dataFormat
        case "WSR"
            timestamp_low=posixtime(datetime('now')); 
            bfee_count=0; Nrx=3; Ntx=1;
            rssi_a=0; rssi_b=0; rssi_c=0;
            noise=129; agc=0;
            perm=[1,2,3]; csi_rate=rate.DesiredRate;
            csi=complex(zeros(Ntx,Nrx,numSubC));
            tv_sec=1; tv_usec=1; frame_count=1;
            mac_RX='0123456789AB'; mac_TX='0123456789CD';
            
            % first overwrite whole struct (prevent data lingering between runs)
            csi_data_RX = struct("timestamp_low",timestamp_low,"bfee_count",bfee_count, ...
                       "Nrx",Nrx,"Ntx",Ntx,"rssi_a",rssi_a,"rssi_b",rssi_b, ...
                       "rssi_c",rssi_c,"noise",noise,"agc",agc,"perm",perm, ...
                       "rate",csi_rate,"csi",csi,"tv_sec",tv_sec,"tv_usec",tv_usec, ...
                       "mac",mac_RX,"frame_count",frame_count);
            csi_data_TX = struct("timestamp_low",timestamp_low,"bfee_count",bfee_count, ...
                       "Nrx",Nrx,"Ntx",Ntx,"rssi_a",rssi_a,"rssi_b",rssi_b, ...
                       "rssi_c",rssi_c,"noise",noise,"agc",agc,"perm",perm, ...
                       "rate",csi_rate,"csi",csi,"tv_sec",tv_sec,"tv_usec",tv_usec, ...
                       "mac",mac_TX,"frame_count",frame_count);
            csi_data_RX(1:i_max) = csi_data_RX(1);
            csi_data_TX(1:i_max) = csi_data_TX(1);
            odom_rx=nan(i_max,9); 


        case "Nexmon"
            




        otherwise
            error("dataformat not properly defined")
    end
    
    % Set initial speed (prevent data lingering between runs)
    linSpeed=[0,0,0]; % Move in X, Y, Z (focus on X,Y)
    rotSpeed=[0,0,0]; % Rot around X, Y, Z (focus on Z) (roll, pitch, yaw)
    msgPub_RX{1}.Linear.X = linSpeed(1);
    msgPub_RX{1}.Linear.Y = linSpeed(2);
    msgPub_RX{1}.Linear.Z = linSpeed(3);
    msgPub_RX{1}.Angular.X = rotSpeed(1);
    msgPub_RX{1}.Angular.Y = rotSpeed(2);
    msgPub_RX{1}.Angular.Z = rotSpeed(3);
    
    %%% Run ROS/Gazebo with the trajectory
    % loop until set time has been reached
    % while rate.TotalElapsedTime < runTime 
    
    % loop over pre-determined trajectory
    for i=1:i_max
        % Receive RX location (subscribe) and store.
        msgSub_RX{controlR}=rosSub_RX{controlR}.receive(1);
        msgSub_TX          =rosSub_TX.receive(1);
        rxPosition =    [msgSub_RX{controlR}.Pose.Pose.Position.X, ...
                         msgSub_RX{controlR}.Pose.Pose.Position.Y, ...
                         msgSub_RX{controlR}.Pose.Pose.Position.Z]; 
        rxOrientation = [msgSub_RX{controlR}.Pose.Pose.Orientation.X, ...
                         msgSub_RX{controlR}.Pose.Pose.Orientation.Y, ...
                         msgSub_RX{controlR}.Pose.Pose.Orientation.Z, ...
                         msgSub_RX{controlR}.Pose.Pose.Orientation.W];
        txPosition =    [msgSub_TX.Pose.Pose.Position.X, ...
                         msgSub_TX.Pose.Pose.Position.Y, ...
                         msgSub_TX.Pose.Pose.Position.Z]; 
    
        % Check ROS odom data
        if any(isnan([txPosition,rxPosition,rxOrientation]))
            warning("NaN value appeared on iteration:" + i);
            continue; % skip this iteration
        end
    
        % Current time in seconds since the UNIX epoch (00:00:00 UTC on 1 January 1970)
        currentTime = posixtime(datetime('now'));
        tv_sec = floor(currentTime);                   % whole seconds
        tv_usec = round((currentTime - tv_sec) * 1e6); % microseconds part
        deltaTime = currentTime - csi_data_RX(1).timestamp_low;
        
        % Store odometry data -> CSV
        odom_rx(i,:)=[currentTime,0,rxPosition,rxOrientation];
        
        % Calculate CSI, make sure to take startPos into account which is not
        % part of robotic odometry.
        [csi_values_RX,csi_values_TX] = csiCalc(txPosition + posStart(1,:),rxPosition + posStart(controlR+1,:), ...
            centerFrequency,bandwidth,numSubC,deltaFrequency,deltaTime,SNR);
    
        % Store CSI data -> DAT file
        csi_data_RX(i).timestamp_low=currentTime; % RX
        csi_data_RX(i).csi(1,1,:)= csi_values_RX;
        csi_data_RX(i).frame_count=i;
        csi_data_RX(i).tv_sec = tv_sec;
        csi_data_RX(i).tv_usec = tv_usec;
        csi_data_TX(i).timestamp_low=currentTime; % TX
        csi_data_TX(i).csi(1,1,:)= csi_values_TX;
        csi_data_TX(i).frame_count=i;
        csi_data_TX(i).tv_sec = tv_sec;
        csi_data_TX(i).tv_usec = tv_usec;
        
        % Update speeds and publish
        msgPub_RX{controlR}.Linear.X = vx(i);
        msgPub_RX{controlR}.Linear.Y = vy(i);
        send(rosPub_RX{controlR},msgPub_RX{controlR});
        
        % Make sure to not overload the system
        waitfor(rate);
    end
    
    % Remove missing data resulting from too large pre-allocation (not really
    % possible with predetermined trajectory)
    if i < i_max
        odom_rx(i:end,:)=[];
        csi_data_RX(i:end)=[];
        fprintf("warning %d from the %d allocated entries\n", i_max-i,i_max)
    end
end
%[text] 
%[text] ### Bartlett Wrapper
function AOA_angle = bartlett_wrapper(csi_data_RX,csi_data_TX,odom_rx,bartlett)
    plot_AoA=bartlett.plot_AoA;         % boolean to show AoA profile plot
    plot_signal=bartlett.plot_signal;   % boolean to show signal properties plot
    plot_odom=bartlett.plot_odom;       % boolean to show odometry/trajectory plot
    plot_popup=bartlett.plot_popup;     % Make plots pop-up from live script
    plot_save=bartlett.plot_save;       % boolean to save all checked plots
    plot_prefix=bartlett.plot_prefix;   % prefix to name the plots
    fixed_yaw=bartlett.fixed_yaw;       % boolean to set manual yaw calculation instead of odom
    simple=bartlett.simple;             % boolean for 2D computation 
    lambda=bartlett.lambda;             % Signal lambda
    trajectory=bartlett.trajectory;     % Trajectory type only for plots

    nbeta = 360;                        % azimuth resolution
    ngamma = 180;                       % elevation resolution
    beta_min = deg2rad(-180);           % minimum azimuth
    beta_max = deg2rad(180);            % maximum azimuth
    gamma_min = deg2rad(-90);           % minimum elevation
    gamma_max = deg2rad(90);            % maximum elevation
    betaList = linspace(beta_min, beta_max, nbeta).';      % azimuth 
    gammaList = linspace(gamma_min, gamma_max, ngamma);    % elevation 
 
    % Get interpolated CSI data out of structs
    h_ij_RX = interpolateCSIold(csi_data_RX);
    h_ij_TX = interpolateCSIold(csi_data_TX);

    % Cancel CFO (No square, already taken into account in Bartlett Estimator)
    h_ij = h_ij_RX .* h_ij_TX;

    
    % The odometry for the roll, pitch and yaw from the nexus robots contains issues. 
    % Therefore, it is better to calculate the yaw manually using the X,Y location data.
    if fixed_yaw
        % Calculate yaw in rad using x and y coordinates
        yawList = atan2(odom_rx(:,4)-odom_rx(1,4),odom_rx(:,3)-odom_rx(1,3));
        yawList(1) = deg2rad(90); % Start at 90 deg based on reference frame
    else
        % Get yaw in rad from odometry (is flawed)
        yawList=odom_rx(:,8); % in rad
        yawList(1) = deg2rad(90); % Start at 90 deg based on reference frame
    end
    
    % rho is distance to starting position, so substract startpos and take 2-norm over each row of the matrix
    rhoList = vecnorm(odom_rx(:,3:5)-odom_rx(1,3:5),2,2); 
    
    % Pitch is in rad, for 2d trajectories it is all zeros.
    pitchList=odom_rx(:,7); 
    
    % Compute the AOA profile
    [AOA_profile, AOA_angle] = bartlett_AOA_estimator(h_ij, yawList, pitchList, ...
        rhoList, lambda, betaList, gammaList, nbeta, ngamma, simple);
    
    % When saving plots, check if suffix is used already to prevent
    % accidental and/or partial overwrite
    if plot_save % only check when saving the plots
        % If any exist already its an issue given inconsitency
        plot_names =  ["_aoa","_signal","_odom"];
        plot_folder = "results/";
        plot_ext =    ".svg";
        for plot_name=plot_names
            % Create the complete paths
            plot_file = strcat(plot_folder,plot_prefix,plot_name,plot_ext);
            if isfile(plot_file)
                error("Plots file suffix already used!")
            end
        end
    end

    %% Plot AOA profile
    if plot_AoA
        figAoA = figure;
        plotAoA = tiledlayout(2,1); % Create a 2x1 tiled chart layout
        
        % Get figure pop-up window
        if plot_popup
            set(gcf,'Visible','on');
        end

        % First plot in the first tile
        ax1 = nexttile; % Get the axes for the first tile
        surf(ax1, rad2deg(betaList), rad2deg(gammaList), AOA_profile.', 'EdgeColor', 'none');
        set(gcf,'Renderer','Zbuffer');            
        xlabel(ax1, 'Azimuth (degrees)');
        ylabel(ax1, 'Elevation (degrees)');
        zlabel(ax1, 'Spectrum (-)');
        zlim(ax1, [0 max(max(AOA_profile))]);
        title(ax1, 'AOA profile (side view)');
        
        % Second plot in the second tile
        ax2 = nexttile; % Get the axes for the second tile
        surf(ax2, rad2deg(betaList), rad2deg(gammaList), AOA_profile.', 'EdgeColor', 'none');
        set(gcf,'Renderer','Zbuffer');
        view(ax2, 2);
        xlabel(ax2, 'Azimuth (degrees)');
        ylabel(ax2, 'Elevation (degrees)');
        zlabel(ax2, 'Estimated spectrum (-)');
        title(ax2, 'AOA profile Top View');
    
        % Overall title for the tiled layout
        title(plotAoA, sprintf('The AoA estimate is %.1f', AOA_angle));
        
        % Save plots (parameters defined above)
        if plot_save
            plot_name = "_aoa";
            plot_file = strcat(plot_folder,plot_prefix,plot_name,plot_ext);
            print(figAoA,plot_file,'-dsvg','-vector');
        end
    end
    
    %% Plot signal properties
    if plot_signal
        % Define x-axis on time or packets
        x_axis = [csi_data_RX.timestamp_low]-csi_data_RX(1).timestamp_low;
        x_label = "Time (s)";
        %x_axis = 1:length(h_ij);
        %x_label = "Packet #";

        figSignal = figure;
        plotSignal = tiledlayout(2, 1); % Create a 2x1 tiled chart layout
        
        % Get figure pop-up window
        if plot_popup
            set(gcf,'Visible','on');
        end
        
        % Amplitude / Magnitude plot in the first tile
        ax1 = nexttile; % Get the axes for the first tile
        csi_amplitude = abs(h_ij);
        scatter(ax1, x_axis, csi_amplitude,".");
        xlabel(ax1, x_label);
        ylabel(ax1, "Amplitude (-)");
        title(ax1, "Signal Amplitude");
        
        % Phase plot in the second tile
        ax2 = nexttile; % Get the axes for the second tile
        csi_phase = (angle(h_ij));
        scatter(ax2, x_axis, csi_phase,".");
        xlabel(ax2, x_label);
        ylabel(ax2, "Phase (rad)");
        ylim([-pi,pi])
        set(gca,'ytick',(-pi:pi/2:pi)) % where to set the tick marks
        set(gca,'yticklabels',{'-\pi','-\pi/2','0','\pi/2','\pi'}) % give them user-defined labels
        title(ax2, "Signal Phase");
        
        % Overall title for the tiled layout
        title(plotSignal, 'CSI Amplitude and Phase');

        % Save plots (parameters defined above)
        if plot_save
            plot_name = "_signal";
            plot_file = strcat(plot_folder,plot_prefix,plot_name,plot_ext);
            print(figSignal,plot_file,'-dsvg','-vector');
        end
    end
    
    % Plot the driven trajectory (starting at 0,0)
    if plot_odom
        % Get radius of circle
        radius=trajectoryRadius(odom_rx(:,3:4),trajectory);
    
        % Plot position data
        figOdom = figure;
        plotOdom = tiledlayout(1, 1); % Get figure pop-up window
        
        % Get figure pop-up window
        if plot_popup
            set(gcf,'Visible','on');
        end

        ax1 = nexttile; % Get the axes for the first tile
        scatter(ax1, odom_rx(:,3)-odom_rx(1,3), ...
            odom_rx(:,4)-odom_rx(1,4),".")
        xlabel(ax1, "X (m)")
        ylabel(ax1, "Y (m)")
        title(ax1, sprintf('The %s radius is %.3f', trajectory, radius))
    
        % Make axis nice size for circles to not look like elipses
        if trajectory == "sin-wave"
            %driven angle
            drive_angle=atan2((odom_rx(end,3)-odom_rx(1,3)),(odom_rx(end,4)-odom_rx(1,4)));
            pbaspect(ax1,[1+abs(sin(drive_angle)), 1+abs(cos(drive_angle)), 1]) % half-square rectangular axis
        elseif trajectory == "line"
            % Make sure a straight line does not look shaky by fixing axis limits
            x_diff= odom_rx(:,3)-odom_rx(1,3);
            x_min = min(x_diff); 
            x_max = max(x_diff);
            x_d   = max((x_max - x_min)/20,0.01);

            y_diff= odom_rx(:,4)-odom_rx(1,4);
            y_min = min(y_diff);
            y_max = max(y_diff);
            y_d   = max((y_max - y_min)/20,0.01);
            
            xlim([x_min-x_d,x_max+x_d]);
            ylim([y_min-y_d,y_max+y_d]);
            pbaspect(ax1,[x_d y_d 1])
        else % circle etc
            pbaspect(ax1,[1 1 1]) % square axis
            axis(ax1,'square')
            axis(ax1,'equal')
        end

        % Overall title for the tiled layout
        title(plotOdom, 'Nexus Robot Odometry');

        % Save plots (parameters defined above)
        if plot_save
            plot_name = "_odom";
            plot_file = strcat(plot_folder,plot_prefix,plot_name,plot_ext);
            print(figOdom,plot_file,'-dsvg','-vector');
        end
    end
end
%[text] #### 
%[text] ### Extract CSI from struct and interpolate
function h_ij = interpolateCSIold(csi_data)
    % Get only CSI data out of structs
    csi_cells = {csi_data.csi};
    csi_data_list = cat(4, csi_cells{:});     % Combine all 1x3x30 complex arrays into 4d array
    csi_data_list = squeeze(csi_data_list(1, 1, 1:30, :)); % Reshape to 2d array Packets*Subcarriers
    
    % Interpolate
    h_ij=nan(size(csi_data_list,2),1);
    xp = 0:29;
    for i=1:size(csi_data_list,2)
        csi_packet=csi_data_list(:,i);
        
        % shifts the jump between consecutive angles by adding multiples of 2*pi until the jump is less than pi.
        fp = unwrap(angle(csi_packet));
    
        % Interpolate CSI subcarriers 1-30 to center carrier 15.5
        p = polyfit(xp, fp, 1);
        interpolated_phase = unwrap(polyval(p, 15.5));
    
        % Take the average magnitude of the signals in the two middle subcarriers
        mag = (abs(csi_packet(15)) + abs(csi_packet(16))) / 2;
        h_ij(i) = mag * exp(1i * interpolated_phase); % is interpolated_h_ij
    end
end
%[text] ### Center Frequency and Bandwidth from Channel
function [centerFrequency,bandwidth] = channelFrequency(channel)
% This functions gives the centerFrequency and bandwith based on the WiFi
% channel number.
% Specify the channel using the following format:
% 2.4GHz 20MHz    1: 1:13
% 2.4GHZ 40MHz  1.5: 1:12.5
%   5GHz 20MHz   32: 4:144
%   5GHz 40MHz   38: 8:142
%   5GHz 80MHz   42:16:138
%   5GHz 160MHz  50:32:114

% https://en.wikipedia.org/wiki/List_of_WLAN_channels
    
    % Check inputs
    if ~any(channel==[1:0.5:13,32,36:2:144])
        error("channel out of range 2.4GHz 1:1:13 or 5GHz 32:2:144 (excl. 34).")
    end   

    % 2.4GHz - 20MHz (1:1:13) - Up to 64 subcarriers
    if channel<=13 && round(channel)==channel
        bandwidth = 20e6;
        centerFrequency = 2412e6 + 5e6 * (channel-1);
    % 2.4GHz - 40MHz (1.5:1:12.5) - Up to 128 subcarriers
    elseif channel<=12.5
        bandwidth = 40e6;
        centerFrequency = 2414e6 + 5e6 * (channel-1.5); %prevent 0.5e6
    % 5GHz - 160MHz (50:32:114) - Up to 512 subcarriers
    elseif mod(channel-18,32)==0
        bandwidth = 160e6;
        centerFrequency = 5250e6 + (160e6 * (channel-50)/32);
    % 5GHz - 80MHz (42:16:138) - Up to 256 subcarriers
    elseif mod(channel-10,16)==0
        bandwidth = 80e6;
        centerFrequency = 5210e6 + ( 80e6 * (channel-42)/16);
    % 5GHz - 40MHz (38:8:142) - Up to 128 subcarriers
    elseif mod(channel-6,8)==0
        bandwidth = 40e6;
        centerFrequency = 5190e6 + ( 40e6 * (channel-38)/8);
    % 5GHz - 20MHz (32:4:144) - Up to 64 subcarriers
    elseif mod(channel,4)==0
        bandwidth = 20e6;
        centerFrequency = 5160e6 + ( 20e6 * (channel-32)/4);
    end
end
%[text] 
%[text] ### CSI Signal Calculation
function csi_data_rx=csiCalc(txPosition,rxPosition,deltaTime,centerFrequency,bandwidth,numSubcarriers,deltaFrequency,SNR)
% Example input:
%txPosition = [0, 0, 0]; % Transmitter position
%rxPosition = [1, 0, 0]; % Receiver position

%%% Example to calculate exact channel information.
% See function channelFrequency for all details.
%channel = 108;
%[centerFrequency,bandwidth] = channelFrequency(channel);
%numSubcarriers = 30;

%%% Example to manually define channel information.
%Center WiFi frequency channel 108 at 5GHz as configured in config.json
%centerFrequency = 5540e6;   % central fequency 5.540GHz (5530-5550)
%bandwidth = 20e6;           % bandwidth 20 MHz
%numSubcarriers = 30;        % subcarriers 30

    % Calculate frequencies
    subcarrierFrequencies = linspace(centerFrequency - bandwidth/2, ...
        centerFrequency + bandwidth/2, numSubcarriers);
    
    %%% Calculate additional info
    %csi_amplitude = abs(csi_data);
    %csi_phase = rad2deg(angle(csi_data));

    % Calculate distance between transmitter and receiver at time t
    distance = norm(txPosition - rxPosition);

    % Calculate wavelength
    lambda = physconst('LightSpeed') ./ subcarrierFrequencies; % Speed of light =~ 3e8 m/s

    % Calculate CSI phase and amplitude
    csi_amplitude = 1 / distance;
    phaseShift = -2 * pi * distance ./ lambda;

    % Calculate the CFO for both nodes
    CFO_RX = 2 * pi * deltaFrequency * deltaTime;
    % OPTIONAL: add small (variable) delay to deltaTime for TX
    %deltaTime = deltaTime + ( -0.0001 + -0.0002*rand(1,1)); 
    %CFO_TX = -2 * pi * deltaFrequency * deltaTime;

    % Add CFO and calculate signal
    csi_data_rx = csi_amplitude .* exp(1i .* (phaseShift + CFO_RX));
    %csi_data_tx = csi_amplitude .* exp(1i .* (phaseShift + CFO_TX));
    
    % Add noise to signal based on noise to signal ratio
    csi_data_rx = awgn(csi_data_rx,SNR,'measured');
    csi_data_rx = awgn(csi_data_rx,SNR,'measured');
    %csi_data_tx = awgn(csi_data_tx,SNR,'measured');
end

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"inline","rightPanelPercent":18.7}
%---
%[control:button:61bf]
%   data: {"label":"Run","run":"Section"}
%---
%[control:button:47b0]
%   data: {"label":"Run","run":"Section"}
%---
%[control:editfield:09c4]
%   data: {"defaultValue":"'192.168.2.143'","label":"IP_laptop","run":"Nothing","valueType":"Char"}
%---
%[control:editfield:6987]
%   data: {"defaultValue":"'192.168.2.99'","label":"IP_desktop","run":"Nothing","valueType":"Char"}
%---
%[control:editfield:72e4]
%   data: {"defaultValue":"\"root\"","label":"User_Remote","run":"Nothing","valueType":"String"}
%---
%[control:dropdown:66ef]
%   data: {"defaultValue":"true","itemLabels":["Local","Remote"],"items":["true","false"],"label":"ROS Core","run":"Nothing"}
%---
%[control:button:290e]
%   data: {"label":"Run","run":"Section"}
%---
%[control:checkbox:2e89]
%   data: {"defaultValue":false,"label":"ROS Debugging","run":"Nothing"}
%---
%[control:checkbox:81d8]
%   data: {"defaultValue":false,"label":"Nexus Car Debugging","run":"Nothing"}
%---
%[control:filebrowser:0ba9]
%   data: {"browserType":"File","defaultValue":"\"pwd + \"\\ground_robot.sdf\"\"","label":"","run":"Nothing"}
%---
%[control:filebrowser:9739]
%   data: {"browserType":"File","defaultValue":"\"pwd + \"\\aerial_robot.urdf\"\"","label":"","run":"Nothing"}
%---
%[control:slider:8e86]
%   data: {"defaultValue":1,"label":"Receivers","max":10,"min":1,"run":"Nothing","runOn":"ValueChanged","step":1}
%---
%[control:editfield:9f3f]
%   data: {"defaultValue":"[ 0, 0, 0, 0 ]","label":"Edit field","run":"Nothing","valueType":"MATLAB code"}
%---
%[control:editfield:1727]
%   data: {"defaultValue":"[ 0, 0, 0, 0 ]","label":"Edit field","run":"Nothing","valueType":"MATLAB code"}
%---
%[control:editfield:6abd]
%   data: {"defaultValue":"[ 0, 0, 0, 0 ]","label":"Edit field","run":"Nothing","valueType":"MATLAB code"}
%---
%[control:editfield:8317]
%   data: {"defaultValue":"[ 0, 0, 0, 0 ]","label":"Edit field","run":"Nothing","valueType":"MATLAB code"}
%---
%[control:editfield:21ed]
%   data: {"defaultValue":"[ 0, 0, 0, 0 ]","label":"Edit field","run":"Nothing","valueType":"MATLAB code"}
%---
%[control:editfield:35eb]
%   data: {"defaultValue":"[ 0, 0, 0, 0 ]","label":"Edit field","run":"Nothing","valueType":"MATLAB code"}
%---
%[control:editfield:5f0d]
%   data: {"defaultValue":"[ 0, 0, 0, 0 ]","label":"Edit field","run":"Nothing","valueType":"MATLAB code"}
%---
%[control:editfield:90eb]
%   data: {"defaultValue":"[ 0, 0, 0, 0 ]","label":"Edit field","run":"Nothing","valueType":"MATLAB code"}
%---
%[control:editfield:6ae6]
%   data: {"defaultValue":"[ 0, 0, 0, 0 ]","label":"Edit field","run":"Nothing","valueType":"MATLAB code"}
%---
%[control:editfield:7d90]
%   data: {"defaultValue":"[ 0, 0, 0, 0 ]","label":"Edit field","run":"Nothing","valueType":"MATLAB code"}
%---
%[control:dropdown:0c8d]
%   data: {"defaultValue":"1","itemLabels":["drone","ground"],"items":["1","0"],"label":"Drop down","run":"Nothing"}
%---
%[control:dropdown:4361]
%   data: {"defaultValue":"1","itemLabels":["drone","ground"],"items":["1","0"],"label":"Drop down","run":"Nothing"}
%---
%[control:dropdown:7f68]
%   data: {"defaultValue":"1","itemLabels":["drone","ground"],"items":["1","0"],"label":"Drop down","run":"Nothing"}
%---
%[control:dropdown:4041]
%   data: {"defaultValue":"1","itemLabels":["drone","ground"],"items":["1","0"],"label":"Drop down","run":"Nothing"}
%---
%[control:dropdown:69ce]
%   data: {"defaultValue":"1","itemLabels":["drone","ground"],"items":["1","0"],"label":"Drop down","run":"Nothing"}
%---
%[control:dropdown:1d17]
%   data: {"defaultValue":"1","itemLabels":["drone","ground"],"items":["1","0"],"label":"Drop down","run":"Nothing"}
%---
%[control:dropdown:486c]
%   data: {"defaultValue":"1","itemLabels":["drone","ground"],"items":["1","0"],"label":"Drop down","run":"Nothing"}
%---
%[control:dropdown:99a5]
%   data: {"defaultValue":"1","itemLabels":["drone","ground"],"items":["1","0"],"label":"Drop down","run":"Nothing"}
%---
%[control:dropdown:3ad9]
%   data: {"defaultValue":"1","itemLabels":["drone","ground"],"items":["1","0"],"label":"Drop down","run":"Nothing"}
%---
%[control:dropdown:1fdc]
%   data: {"defaultValue":"1","itemLabels":["drone","ground"],"items":["1","0"],"label":"Drop down","run":"Nothing"}
%---
%[control:editfield:11d2]
%   data: {"defaultValue":"'0123456789AB'","label":"Edit field","run":"Nothing","valueType":"Char"}
%---
%[control:editfield:2731]
%   data: {"defaultValue":"'0123456789AB'","label":"Edit field","run":"Nothing","valueType":"Char"}
%---
%[control:editfield:9b1d]
%   data: {"defaultValue":"'0123456789AB'","label":"Edit field","run":"Nothing","valueType":"Char"}
%---
%[control:editfield:3834]
%   data: {"defaultValue":"'0123456789AB'","label":"Edit field","run":"Nothing","valueType":"Char"}
%---
%[control:editfield:593f]
%   data: {"defaultValue":"'0123456789AB'","label":"Edit field","run":"Nothing","valueType":"Char"}
%---
%[control:editfield:7e71]
%   data: {"defaultValue":"'0123456789AB'","label":"Edit field","run":"Nothing","valueType":"Char"}
%---
%[control:editfield:9279]
%   data: {"defaultValue":"'0123456789AB'","label":"Edit field","run":"Nothing","valueType":"Char"}
%---
%[control:editfield:958e]
%   data: {"defaultValue":"'0123456789AB'","label":"Edit field","run":"Nothing","valueType":"Char"}
%---
%[control:editfield:6122]
%   data: {"defaultValue":"'0123456789AB'","label":"Edit field","run":"Nothing","valueType":"Char"}
%---
%[control:editfield:9688]
%   data: {"defaultValue":"'0123456789AB'","label":"Edit field","run":"Nothing","valueType":"Char"}
%---
%[control:editfield:8f87]
%   data: {"defaultValue":0,"label":"channel","run":"Nothing","valueType":"Double"}
%---
%[control:editfield:2a2f]
%   data: {"defaultValue":" 30","label":"numSubC","run":"Nothing","valueType":"MATLAB code"}
%---
%[control:dropdown:709c]
%   data: {"defaultValue":"\"WSR\"","itemLabels":["WSR","Nexmon"],"items":["\"WSR\"","\"Nexmon\""],"label":"dataFormat","run":"Nothing"}
%---
%[control:editfield:7e54]
%   data: {"defaultValue":0,"label":"Edit field","run":"Nothing","valueType":"Double"}
%---
%[control:editfield:86b4]
%   data: {"defaultValue":0,"label":"num_points","run":"Section","valueType":"Double"}
%---
%[control:button:0938]
%   data: {"label":"Run","run":"Section"}
%---
%[control:checkbox:1e72]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:83d3]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:3507]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:1cdb]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:8506]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:7236]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:89b7]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:3e66]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:9ec1]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:8d12]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:86ff]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:33d6]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:7995]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:4b98]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:683c]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:1f24]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:6cd0]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:6afb]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:19ff]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:9028]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:8766]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:7809]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:40a3]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:518b]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:1427]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:1e5e]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:9b4d]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:23a7]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:8aca]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:5f2f]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:3a2a]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:7a78]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:261d]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:5457]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:84af]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:61c4]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:8498]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:21fb]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:6a6b]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:09ff]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:1297]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:35e7]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:71f9]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:29d7]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:8eb1]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:dropdown:9d43]
%   data: {"defaultValue":"trajTypeOptions(1)","itemLabels":["static","circle","sin-wave","line","rendezvous","random-walk"],"items":["trajTypeOptions(1)","trajTypeOptions(2)","trajTypeOptions(3)","trajTypeOptions(4)","trajTypeOptions(5)","trajTypeOptions(6)"],"itemsVariable":"trajTypeOptions","label":"trajectory type","run":"Nothing"}
%---
%[control:dropdown:83f2]
%   data: {"defaultValue":"trajTypeOptions(1)","itemLabels":["static","circle","sin-wave","line","rendezvous","random-walk"],"items":["trajTypeOptions(1)","trajTypeOptions(2)","trajTypeOptions(3)","trajTypeOptions(4)","trajTypeOptions(5)","trajTypeOptions(6)"],"itemsVariable":"trajTypeOptions","label":"trajectory type","run":"Nothing"}
%---
%[control:dropdown:09a6]
%   data: {"defaultValue":"trajTypeOptions(1)","itemLabels":["static","circle","sin-wave","line","rendezvous","random-walk"],"items":["trajTypeOptions(1)","trajTypeOptions(2)","trajTypeOptions(3)","trajTypeOptions(4)","trajTypeOptions(5)","trajTypeOptions(6)"],"itemsVariable":"trajTypeOptions","label":"trajectory type","run":"Nothing"}
%---
%[control:dropdown:2357]
%   data: {"defaultValue":"trajTypeOptions(1)","itemLabels":["static","circle","sin-wave","line","rendezvous","random-walk"],"items":["trajTypeOptions(1)","trajTypeOptions(2)","trajTypeOptions(3)","trajTypeOptions(4)","trajTypeOptions(5)","trajTypeOptions(6)"],"itemsVariable":"trajTypeOptions","label":"trajectory type","run":"Nothing"}
%---
%[control:dropdown:4e01]
%   data: {"defaultValue":"trajTypeOptions(1)","itemLabels":["static","circle","sin-wave","line","rendezvous","random-walk"],"items":["trajTypeOptions(1)","trajTypeOptions(2)","trajTypeOptions(3)","trajTypeOptions(4)","trajTypeOptions(5)","trajTypeOptions(6)"],"itemsVariable":"trajTypeOptions","label":"trajectory type","run":"Nothing"}
%---
%[control:dropdown:0bcb]
%   data: {"defaultValue":"trajTypeOptions(1)","itemLabels":["static","circle","sin-wave","line","rendezvous","random-walk"],"items":["trajTypeOptions(1)","trajTypeOptions(2)","trajTypeOptions(3)","trajTypeOptions(4)","trajTypeOptions(5)","trajTypeOptions(6)"],"itemsVariable":"trajTypeOptions","label":"trajectory type","run":"Nothing"}
%---
%[control:dropdown:8bfb]
%   data: {"defaultValue":"trajTypeOptions(1)","itemLabels":["static","circle","sin-wave","line","rendezvous","random-walk"],"items":["trajTypeOptions(1)","trajTypeOptions(2)","trajTypeOptions(3)","trajTypeOptions(4)","trajTypeOptions(5)","trajTypeOptions(6)"],"itemsVariable":"trajTypeOptions","label":"trajectory type","run":"Nothing"}
%---
%[control:dropdown:9930]
%   data: {"defaultValue":"trajTypeOptions(1)","itemLabels":["static","circle","sin-wave","line","rendezvous","random-walk"],"items":["trajTypeOptions(1)","trajTypeOptions(2)","trajTypeOptions(3)","trajTypeOptions(4)","trajTypeOptions(5)","trajTypeOptions(6)"],"itemsVariable":"trajTypeOptions","label":"trajectory type","run":"Nothing"}
%---
%[control:dropdown:6e23]
%   data: {"defaultValue":"trajTypeOptions(1)","itemLabels":["static","circle","sin-wave","line","rendezvous","random-walk"],"items":["trajTypeOptions(1)","trajTypeOptions(2)","trajTypeOptions(3)","trajTypeOptions(4)","trajTypeOptions(5)","trajTypeOptions(6)"],"itemsVariable":"trajTypeOptions","label":"trajectory type","run":"Nothing"}
%---
%[control:dropdown:48a8]
%   data: {"defaultValue":"trajTypeOptions(1)","itemLabels":["static","circle","sin-wave","line","rendezvous","random-walk"],"items":["trajTypeOptions(1)","trajTypeOptions(2)","trajTypeOptions(3)","trajTypeOptions(4)","trajTypeOptions(5)","trajTypeOptions(6)"],"itemsVariable":"trajTypeOptions","label":"trajectory type","run":"Nothing"}
%---
%[control:dropdown:057b]
%   data: {"defaultValue":"robotList(1)","itemLabels":["1","2","3","4","5","6"],"items":["robotList(1)","robotList(2)","robotList(3)","robotList(4)","robotList(5)","robotList(6)"],"itemsVariable":"robotList","label":"Drop down","run":"Nothing"}
%---
%[control:dropdown:936a]
%   data: {"defaultValue":"robotList(1)","itemLabels":["1","2","3","4","5","6"],"items":["robotList(1)","robotList(2)","robotList(3)","robotList(4)","robotList(5)","robotList(6)"],"itemsVariable":"robotList","label":"Drop down","run":"Nothing"}
%---
%[control:dropdown:2009]
%   data: {"defaultValue":"robotList(1)","itemLabels":["1","2","3","4","5","6"],"items":["robotList(1)","robotList(2)","robotList(3)","robotList(4)","robotList(5)","robotList(6)"],"itemsVariable":"robotList","label":"Drop down","run":"Nothing"}
%---
%[control:dropdown:4f44]
%   data: {"defaultValue":"robotList(1)","itemLabels":["1","2","3","4","5","6"],"items":["robotList(1)","robotList(2)","robotList(3)","robotList(4)","robotList(5)","robotList(6)"],"itemsVariable":"robotList","label":"Drop down","run":"Nothing"}
%---
%[control:dropdown:9b2e]
%   data: {"defaultValue":"robotList(1)","itemLabels":["1","2","3","4","5","6"],"items":["robotList(1)","robotList(2)","robotList(3)","robotList(4)","robotList(5)","robotList(6)"],"itemsVariable":"robotList","label":"Drop down","run":"Nothing"}
%---
%[control:dropdown:0bf8]
%   data: {"defaultValue":"robotList(1)","itemLabels":["1","2","3","4","5","6"],"items":["robotList(1)","robotList(2)","robotList(3)","robotList(4)","robotList(5)","robotList(6)"],"itemsVariable":"robotList","label":"Drop down","run":"Nothing"}
%---
%[control:dropdown:4738]
%   data: {"defaultValue":"robotList(1)","itemLabels":["1","2","3","4","5","6"],"items":["robotList(1)","robotList(2)","robotList(3)","robotList(4)","robotList(5)","robotList(6)"],"itemsVariable":"robotList","label":"Drop down","run":"Nothing"}
%---
%[control:dropdown:5c11]
%   data: {"defaultValue":"robotList(1)","itemLabels":["1","2","3","4","5","6"],"items":["robotList(1)","robotList(2)","robotList(3)","robotList(4)","robotList(5)","robotList(6)"],"itemsVariable":"robotList","label":"Drop down","run":"Nothing"}
%---
%[control:dropdown:277c]
%   data: {"defaultValue":"robotList(1)","itemLabels":["1","2","3","4","5","6"],"items":["robotList(1)","robotList(2)","robotList(3)","robotList(4)","robotList(5)","robotList(6)"],"itemsVariable":"robotList","label":"Drop down","run":"Nothing"}
%---
%[control:dropdown:12a9]
%   data: {"defaultValue":"robotList(1)","itemLabels":["1","2","3","4","5","6"],"items":["robotList(1)","robotList(2)","robotList(3)","robotList(4)","robotList(5)","robotList(6)"],"itemsVariable":"robotList","label":"Drop down","run":"Nothing"}
%---
%[control:checkbox:7c51]
%   data: {"defaultValue":false,"label":"trajOmni","run":"Nothing"}
%---
%[control:checkbox:22ab]
%   data: {"defaultValue":false,"label":"trajOmni","run":"Nothing"}
%---
%[control:checkbox:878e]
%   data: {"defaultValue":false,"label":"trajOmni","run":"Nothing"}
%---
%[control:checkbox:11b5]
%   data: {"defaultValue":false,"label":"trajOmni","run":"Nothing"}
%---
%[control:checkbox:8188]
%   data: {"defaultValue":false,"label":"trajOmni","run":"Nothing"}
%---
%[control:checkbox:974f]
%   data: {"defaultValue":false,"label":"trajOmni","run":"Nothing"}
%---
%[control:checkbox:8284]
%   data: {"defaultValue":false,"label":"trajOmni","run":"Nothing"}
%---
%[control:checkbox:2d71]
%   data: {"defaultValue":false,"label":"trajOmni","run":"Nothing"}
%---
%[control:checkbox:2eb1]
%   data: {"defaultValue":false,"label":"trajOmni","run":"Nothing"}
%---
%[control:checkbox:903a]
%   data: {"defaultValue":false,"label":"trajOmni","run":"Nothing"}
%---
%[control:editfield:404b]
%   data: {"defaultValue":0,"label":"","run":"Nothing","valueType":"Double"}
%---
%[control:editfield:4f4f]
%   data: {"defaultValue":0,"label":"Edit field","run":"Nothing","valueType":"Double"}
%---
%[control:editfield:9b11]
%   data: {"defaultValue":0,"label":"Edit field","run":"Nothing","valueType":"Double"}
%---
%[control:editfield:4f42]
%   data: {"defaultValue":0,"label":"Edit field","run":"Nothing","valueType":"Double"}
%---
%[control:editfield:28c3]
%   data: {"defaultValue":0,"label":"Edit field","run":"Nothing","valueType":"Double"}
%---
%[control:editfield:18b8]
%   data: {"defaultValue":0,"label":"","run":"Nothing","valueType":"Double"}
%---
%[control:editfield:1a33]
%   data: {"defaultValue":0,"label":"Edit field","run":"Nothing","valueType":"Double"}
%---
%[control:editfield:574a]
%   data: {"defaultValue":0,"label":"Edit field","run":"Nothing","valueType":"Double"}
%---
%[control:editfield:391d]
%   data: {"defaultValue":0,"label":"Edit field","run":"Nothing","valueType":"Double"}
%---
%[control:editfield:372a]
%   data: {"defaultValue":0,"label":"Edit field","run":"Nothing","valueType":"Double"}
%---
%[control:editfield:1994]
%   data: {"defaultValue":0,"label":"","run":"Nothing","valueType":"Double"}
%---
%[control:editfield:27ea]
%   data: {"defaultValue":0,"label":"Edit field","run":"Nothing","valueType":"Double"}
%---
%[control:editfield:1890]
%   data: {"defaultValue":0,"label":"Edit field","run":"Nothing","valueType":"Double"}
%---
%[control:editfield:6b17]
%   data: {"defaultValue":0,"label":"Edit field","run":"Nothing","valueType":"Double"}
%---
%[control:editfield:2e35]
%   data: {"defaultValue":0,"label":"Edit field","run":"Nothing","valueType":"Double"}
%---
%[control:editfield:5ad3]
%   data: {"defaultValue":0,"label":"","run":"Nothing","valueType":"Double"}
%---
%[control:editfield:449c]
%   data: {"defaultValue":0,"label":"Edit field","run":"Nothing","valueType":"Double"}
%---
%[control:editfield:35f4]
%   data: {"defaultValue":0,"label":"Edit field","run":"Nothing","valueType":"Double"}
%---
%[control:editfield:1ec2]
%   data: {"defaultValue":0,"label":"Edit field","run":"Nothing","valueType":"Double"}
%---
%[control:editfield:4431]
%   data: {"defaultValue":0,"label":"Edit field","run":"Nothing","valueType":"Double"}
%---
%[control:editfield:259d]
%   data: {"defaultValue":0,"label":"Edit field","run":"Nothing","valueType":"Double"}
%---
%[control:editfield:6bb0]
%   data: {"defaultValue":0,"label":"Edit field","run":"Nothing","valueType":"Double"}
%---
%[control:editfield:6d07]
%   data: {"defaultValue":0,"label":"Edit field","run":"Nothing","valueType":"Double"}
%---
%[control:editfield:00ff]
%   data: {"defaultValue":0,"label":"Edit field","run":"Nothing","valueType":"Double"}
%---
%[control:editfield:405f]
%   data: {"defaultValue":0,"label":"Edit field","run":"Nothing","valueType":"Double"}
%---
%[control:editfield:7ee9]
%   data: {"defaultValue":0,"label":"Edit field","run":"Nothing","valueType":"Double"}
%---
%[control:editfield:591c]
%   data: {"defaultValue":0,"label":"Edit field","run":"Nothing","valueType":"Double"}
%---
%[control:editfield:10f6]
%   data: {"defaultValue":0,"label":"Edit field","run":"Nothing","valueType":"Double"}
%---
%[control:editfield:1e8c]
%   data: {"defaultValue":0,"label":"Edit field","run":"Nothing","valueType":"Double"}
%---
%[control:editfield:472b]
%   data: {"defaultValue":0,"label":"Edit field","run":"Nothing","valueType":"Double"}
%---
%[control:editfield:6a3f]
%   data: {"defaultValue":0,"label":"deltaFrequency","run":"Nothing","valueType":"Double"}
%---
%[control:editfield:2cb3]
%   data: {"defaultValue":0,"label":"deltaFrequency","run":"Nothing","valueType":"Double"}
%---
%[control:editfield:1330]
%   data: {"defaultValue":0,"label":"deltaFrequency","run":"Nothing","valueType":"Double"}
%---
%[control:editfield:7292]
%   data: {"defaultValue":0,"label":"deltaFrequency","run":"Nothing","valueType":"Double"}
%---
%[control:editfield:0d54]
%   data: {"defaultValue":0,"label":"deltaFrequency","run":"Nothing","valueType":"Double"}
%---
%[control:editfield:5866]
%   data: {"defaultValue":0,"label":"deltaFrequency","run":"Nothing","valueType":"Double"}
%---
%[control:editfield:8f5f]
%   data: {"defaultValue":0,"label":"deltaFrequency","run":"Nothing","valueType":"Double"}
%---
%[control:editfield:824c]
%   data: {"defaultValue":0,"label":"deltaFrequency","run":"Nothing","valueType":"Double"}
%---
%[control:editfield:0dd7]
%   data: {"defaultValue":0,"label":"deltaFrequency","run":"Nothing","valueType":"Double"}
%---
%[control:editfield:35f1]
%   data: {"defaultValue":0,"label":"deltaFrequency","run":"Nothing","valueType":"Double"}
%---
%[control:editfield:8d21]
%   data: {"defaultValue":0,"label":"SNR","run":"Nothing","valueType":"Double"}
%---
%[control:editfield:25cb]
%   data: {"defaultValue":0,"label":"SNR","run":"Nothing","valueType":"Double"}
%---
%[control:checkbox:2135]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:42fc]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:24b0]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:2912]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:49bb]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:38ca]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:9f3f]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:5e43]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:5256]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:3978]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:0277]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:91c3]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:8c8d]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:8389]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:7c16]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:2088]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:8325]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:3050]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:759b]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:5e84]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:74f5]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:08c5]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:8e20]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:7b49]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:5443]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:67f7]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:6784]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:0620]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:4b56]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:5b42]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:38ab]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:7e4a]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:43b6]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:33c2]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:8685]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:97c4]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:9785]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:117a]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:1b22]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:6e04]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:66b3]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:0324]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:574c]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:6c08]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:0546]
%   data: {"defaultValue":false,"label":"Check box","run":"Nothing"}
%---
%[control:checkbox:9994]
%   data: {"defaultValue":false,"label":"plotSignal","run":"Nothing"}
%---
%[control:checkbox:5641]
%   data: {"defaultValue":false,"label":"plotSignal","run":"Nothing"}
%---
%[control:checkbox:10f0]
%   data: {"defaultValue":false,"label":"plotSignal","run":"Nothing"}
%---
%[control:checkbox:1e2b]
%   data: {"defaultValue":false,"label":"plotSignal","run":"Nothing"}
%---
%[control:dropdown:5ceb]
%   data: {"defaultValue":"\"on\"","itemLabels":["True","False"],"items":["\"on\"","\"off\""],"label":"plotVisible","run":"Nothing"}
%---
%[control:editfield:6f21]
%   data: {"defaultValue":0,"label":"nbeta","run":"Nothing","valueType":"Double"}
%---
%[control:editfield:0560]
%   data: {"defaultValue":0,"label":"ngamma","run":"Nothing","valueType":"Double"}
%---
%[control:slider:00e0]
%   data: {"defaultValue":-180,"label":"beta_min","max":180,"min":-180,"run":"Nothing","runOn":"ValueChanged","step":15}
%---
%[control:slider:7978]
%   data: {"defaultValue":180,"label":"beta_max","max":180,"min":-180,"run":"Nothing","runOn":"ValueChanged","step":15}
%---
%[control:slider:04f1]
%   data: {"defaultValue":0,"label":"gamma_min","max":180,"min":0,"run":"Nothing","runOn":"ValueChanged","step":15}
%---
%[control:slider:6864]
%   data: {"defaultValue":180,"label":"gamma_max","max":180,"min":0,"run":"Nothing","runOn":"ValueChanged","step":15}
%---
%[control:filebrowser:79b9]
%   data: {"browserType":"Folder","defaultValue":"\"\"","label":"","run":"Nothing"}
%---
%[control:editfield:1bb9]
%   data: {"defaultValue":0,"label":"nbeta","run":"Nothing","valueType":"Double"}
%---
%[control:editfield:64f4]
%   data: {"defaultValue":0,"label":"ngamma","run":"Nothing","valueType":"Double"}
%---
%[control:slider:4267]
%   data: {"defaultValue":-180,"label":"beta_min","max":180,"min":-180,"run":"Nothing","runOn":"ValueChanged","step":15}
%---
%[control:slider:4ad9]
%   data: {"defaultValue":180,"label":"beta_max","max":180,"min":-180,"run":"Nothing","runOn":"ValueChanged","step":15}
%---
%[control:slider:3646]
%   data: {"defaultValue":0,"label":"gamma_min","max":180,"min":0,"run":"Nothing","runOn":"ValueChanged","step":15}
%---
%[control:slider:66a3]
%   data: {"defaultValue":180,"label":"gamma_max","max":180,"min":0,"run":"Nothing","runOn":"ValueChanged","step":15}
%---
%[control:button:059b]
%   data: {"label":"Run","run":"Section"}
%---
%[control:checkbox:2114]
%   data: {"defaultValue":false,"label":"plot_AoA","run":"Nothing"}
%---
%[control:checkbox:04b4]
%   data: {"defaultValue":false,"label":"plot_signal","run":"Nothing"}
%---
%[control:checkbox:50b1]
%   data: {"defaultValue":true,"label":"plot_results","run":"Nothing"}
%---
%[control:checkbox:3740]
%   data: {"defaultValue":false,"label":"plot_popup","run":"Nothing"}
%---
%[control:checkbox:3edf]
%   data: {"defaultValue":false,"label":"plot_save","run":"Nothing"}
%---
%[control:editfield:1a08]
%   data: {"defaultValue":"\"test\"","label":"plot_suffix","run":"Nothing","valueType":"String"}
%---
%[control:checkbox:4697]
%   data: {"defaultValue":true,"label":"fixed_yaw","run":"Nothing"}
%---
%[control:checkbox:4aab]
%   data: {"defaultValue":true,"label":"simple","run":"Nothing"}
%---
%[control:checkbox:4303]
%   data: {"defaultValue":false,"label":"sendfiles","run":"Nothing"}
%---
%[control:editfield:45d5]
%   data: {"defaultValue":"'alex'","label":"sendusr","run":"Section","valueType":"Char"}
%---
%[control:editfield:7efa]
%   data: {"defaultValue":"'\/home\/alex\/wsr_data'","label":"senddir","run":"Section","valueType":"Char"}
%---
%[output:60ae8474]
%   data: {"dataType":"text","outputData":{"text":"The value of the ROS_HOSTNAME environment variable, 192.168.137.1, will be used to set the advertised address for the ROS node.\nInitializing global node \/matlab_global_node_16431 with NodeURI http:\/\/192.168.137.1:55271\/ and MasterURI http:\/\/192.168.137.98:11311.\n\nRUN ON REMOTE PC\n$ export ROS_IP=192.168.137.98\n$ export ROS_HOSTNAME=192.168.137.98\n$ export ROS_MASTER_URI=http:\/\/192.168.137.98:11311\n$ rosrun gazebo_ros gazebo\n","truncated":false}}
%---
%[output:2e96ce54]
%   data: {"dataType":"text","outputData":{"text":"<robot name=\"turtlebot3_waffle_pi\">\\n<!-- Init colour -->\\n<material name=\"black\">\\n<color rgba=\"0.0 0.0 0.0 1.0\"\/>\\n<\/material>\\n<material name=\"dark\">\\n<color rgba=\"0.3 0.3 0.3 1.0\"\/>\\n<\/material>\\n<material name=\"light_black\">\\n<color rgba=\"0.4 0.4 0.4 1.0\"\/>\\n<\/material>\\n<material name=\"blue\">\\n<color rgba=\"0.0 0.0 0.8 1.0\"\/>\\n<\/material>\\n<material name=\"green\">\\n<color rgba=\"0.0 0.8 0.0 1.0\"\/>\\n<\/material>\\n<material name=\"grey\">\\n<color rgba=\"0.5 0.5 0.5 1.0\"\/>\\n<\/material>\\n<material name=\"orange\">\\n<color rgba=\"1.0 0.4235294117647059 0.0392156862745098 1.0\"\/>\\n<\/material>\\n<material name=\"brown\">\\n<color rgba=\"0.8705882352941177 0.8117647058823529 0.7647058823529411 1.0\"\/>\\n<\/material>\\n<material name=\"red\">\\n<color rgba=\"0.8 0.0 0.0 1.0\"\/>\\n<\/material>\\n<material name=\"white\">\\n<color rgba=\"1.0 1.0 1.0 1.0\"\/>\\n<\/material>\\n<gazebo reference=\"base_link\">\\n<material>Gazebo\/DarkGrey<\/material>\\n<\/gazebo>\\n<gazebo reference=\"wheel_left_link\">\\n<mu1>0.1<\/mu1>\\n<mu2>0.1<\/mu2>\\n<kp>500000.0<\/kp>\\n<kd>10.0<\/kd>\\n<minDepth>0.001<\/minDepth>\\n<maxVel>0.1<\/maxVel>\\n<fdir1>1 0 0<\/fdir1>\\n<material>Gazebo\/FlatBlack<\/material>\\n<\/gazebo>\\n<gazebo reference=\"wheel_right_link\">\\n<mu1>0.1<\/mu1>\\n<mu2>0.1<\/mu2>\\n<kp>500000.0<\/kp>\\n<kd>10.0<\/kd>\\n<minDepth>0.001<\/minDepth>\\n<maxVel>0.1<\/maxVel>\\n<fdir1>1 0 0<\/fdir1>\\n<material>Gazebo\/FlatBlack<\/material>\\n<\/gazebo>\\n<gazebo reference=\"caster_back_right_link\">\\n<mu1>0.1<\/mu1>\\n<mu2>0.1<\/mu2>\\n<kp>1000000.0<\/kp>\\n<kd>100.0<\/kd>\\n<minDepth>0.001<\/minDepth>\\n<maxVel>1.0<\/maxVel>\\n<material>Gazebo\/FlatBlack<\/material>\\n<\/gazebo>\\n<gazebo reference=\"caster_back_left_link\">\\n<mu1>0.1<\/mu1>\\n<mu2>0.1<\/mu2>\\n<kp>1000000.0<\/kp>\\n<kd>100.0<\/kd>\\n<minDepth>0.001<\/minDepth>\\n<maxVel>1.0<\/maxVel>\\n<material>Gazebo\/FlatBlack<\/material>\\n<\/gazebo>\\n<gazebo reference=\"imu_link\">\\n<sensor name=\"imu\" type=\"imu\">\\n<always_on>true<\/always_on>\\n<visualize>false<\/visualize>\\n<\/sensor>\\n<material>Gazebo\/Grey<\/material>\\n<\/gazebo>\\n<gazebo>\\n<plugin filename=\"libgazebo_ros_diff_drive.so\" name=\"turtlebot3_waffle_pi_controller\">\\n<commandTopic>cmd_vel<\/commandTopic>\\n<odometryTopic>odom<\/odometryTopic>\\n<odometryFrame>odom<\/odometryFrame>\\n<odometrySource>world<\/odometrySource>\\n<publishOdomTF>true<\/publishOdomTF>\\n<robotBaseFrame>base_footprint<\/robotBaseFrame>\\n<publishWheelTF>false<\/publishWheelTF>\\n<publishTf>true<\/publishTf>\\n<publishWheelJointState>true<\/publishWheelJointState>\\n<legacyMode>false<\/legacyMode>\\n<updateRate>100<\/updateRate>\\n<leftJoint>wheel_left_joint<\/leftJoint>\\n<rightJoint>wheel_right_joint<\/rightJoint>\\n<wheelSeparation>0.287<\/wheelSeparation>\\n<wheelDiameter>0.066<\/wheelDiameter>\\n<wheelAcceleration>1<\/wheelAcceleration>\\n<wheelTorque>10<\/wheelTorque>\\n<rosDebugLevel>na<\/rosDebugLevel>\\n<\/plugin>\\n<\/gazebo>\\n<gazebo>\\n<plugin filename=\"libgazebo_ros_imu.so\" name=\"imu_plugin\">\\n<alwaysOn>true<\/alwaysOn>\\n<bodyName>imu_link<\/bodyName>\\n<frameName>imu_link<\/frameName>\\n<topicName>imu<\/topicName>\\n<serviceName>imu_service<\/serviceName>\\n<gaussianNoise>0.0<\/gaussianNoise>\\n<updateRate>0<\/updateRate>\\n<imu>\\n<noise>\\n<type>gaussian<\/type>\\n<rate>\\n<mean>0.0<\/mean>\\n<stddev>2e-4<\/stddev>\\n<bias_mean>0.0000075<\/bias_mean>\\n<bias_stddev>0.0000008<\/bias_stddev>\\n<\/rate>\\n<accel>\\n<mean>0.0<\/mean>\\n<stddev>1.7e-2<\/stddev>\\n<bias_mean>0.1<\/bias_mean>\\n<bias_stddev>0.001<\/bias_stddev>\\n<\/accel>\\n<\/noise>\\n<\/imu>\\n<\/plugin>\\n<\/gazebo>\\n<link name=\"base_footprint\"\/>\\n<joint name=\"base_joint\" type=\"fixed\">\\n<parent link=\"base_footprint\"\/>\\n<child link=\"base_link\"\/>\\n<origin rpy=\"0 0 0\" xyz=\"0 0 0.010\"\/>\\n<\/joint>\\n<link name=\"base_link\">\\n<visual>\\n<origin rpy=\"0 0 0\" xyz=\"-0.064 0 0.0\"\/>\\n<geometry>\\n<mesh filename=\"package:\/\/turtlebot3_description\/meshes\/bases\/waffle_pi_base.stl\" scale=\"0.001 0.001 0.001\"\/>\\n<\/geometry>\\n<material name=\"light_black\"\/>\\n<\/visual>\\n<collision>\\n<origin rpy=\"0 0 0\" xyz=\"-0.064 0 0.047\"\/>\\n<geometry>\\n<box size=\"0.266 0.266 0.094\"\/>\\n<\/geometry>\\n<\/collision>\\n<inertial>\\n<origin rpy=\"0 0 0\" xyz=\"0 0 0\"\/>\\n<mass value=\"1.3729096e+00\"\/>\\n<inertia ixx=\"8.7002718e-03\" ixy=\"-4.7576583e-05\" ixz=\"1.1160499e-04\" iyy=\"8.6195418e-03\" iyz=\"-3.5422299e-06\" izz=\"1.4612727e-02\"\/>\\n<\/inertial>\\n<\/link>\\n<joint name=\"wheel_left_joint\" type=\"continuous\">\\n<parent link=\"base_link\"\/>\\n<child link=\"wheel_left_link\"\/>\\n<origin rpy=\"-1.57 0 0\" xyz=\"0.0 0.144 0.023\"\/>\\n<axis xyz=\"0 0 1\"\/>\\n<\/joint>\\n<link name=\"wheel_left_link\">\\n<visual>\\n<origin rpy=\"1.57 0 0\" xyz=\"0 0 0\"\/>\\n<geometry>\\n<mesh filename=\"package:\/\/turtlebot3_description\/meshes\/wheels\/left_tire.stl\" scale=\"0.001 0.001 0.001\"\/>\\n<\/geometry>\\n<material name=\"dark\"\/>\\n<\/visual>\\n<collision>\\n<origin rpy=\"0 0 0\" xyz=\"0 0 0\"\/>\\n<geometry>\\n<cylinder length=\"0.018\" radius=\"0.033\"\/>\\n<\/geometry>\\n<\/collision>\\n<inertial>\\n<origin xyz=\"0 0 0\"\/>\\n<mass value=\"2.8498940e-02\"\/>\\n<inertia ixx=\"1.1175580e-05\" ixy=\"-4.2369783e-11\" ixz=\"-5.9381719e-09\" iyy=\"1.1192413e-05\" iyz=\"-1.4400107e-11\" izz=\"2.0712558e-05\"\/>\\n<\/inertial>\\n<\/link>\\n<joint name=\"wheel_right_joint\" type=\"continuous\">\\n<parent link=\"base_link\"\/>\\n<child link=\"wheel_right_link\"\/>\\n<origin rpy=\"-1.57 0 0\" xyz=\"0.0 -0.144 0.023\"\/>\\n<axis xyz=\"0 0 1\"\/>\\n<\/joint>\\n<link name=\"wheel_right_link\">\\n<visual>\\n<origin rpy=\"1.57 0 0\" xyz=\"0 0 0\"\/>\\n<geometry>\\n<mesh filename=\"package:\/\/turtlebot3_description\/meshes\/wheels\/right_tire.stl\" scale=\"0.001 0.001 0.001\"\/>\\n<\/geometry>\\n<material name=\"dark\"\/>\\n<\/visual>\\n<collision>\\n<origin rpy=\"0 0 0\" xyz=\"0 0 0\"\/>\\n<geometry>\\n<cylinder length=\"0.018\" radius=\"0.033\"\/>\\n<\/geometry>\\n<\/collision>\\n<inertial>\\n<origin xyz=\"0 0 0\"\/>\\n<mass value=\"2.8498940e-02\"\/>\\n<inertia ixx=\"1.1175580e-05\" ixy=\"-4.2369783e-11\" ixz=\"-5.9381719e-09\" iyy=\"1.1192413e-05\" iyz=\"-1.4400107e-11\" izz=\"2.0712558e-05\"\/>\\n<\/inertial>\\n<\/link>\\n<joint name=\"caster_back_right_joint\" type=\"fixed\">\\n<parent link=\"base_link\"\/>\\n<child link=\"caster_back_right_link\"\/>\\n<origin rpy=\"-1.57 0 0\" xyz=\"-0.177 -0.064 -0.004\"\/>\\n<\/joint>\\n<link name=\"caster_back_right_link\">\\n<collision>\\n<origin rpy=\"0 0 0\" xyz=\"0 0.001 0\"\/>\\n<geometry>\\n<box size=\"0.030 0.009 0.020\"\/>\\n<\/geometry>\\n<\/collision>\\n<inertial>\\n<origin xyz=\"0 0 0\"\/>\\n<mass value=\"0.005\"\/>\\n<inertia ixx=\"0.001\" ixy=\"0.0\" ixz=\"0.0\" iyy=\"0.001\" iyz=\"0.0\" izz=\"0.001\"\/>\\n<\/inertial>\\n<\/link>\\n<joint name=\"caster_back_left_joint\" type=\"fixed\">\\n<parent link=\"base_link\"\/>\\n<child link=\"caster_back_left_link\"\/>\\n<origin rpy=\"-1.57 0 0\" xyz=\"-0.177 0.064 -0.004\"\/>\\n<\/joint>\\n<link name=\"caster_back_left_link\">\\n<collision>\\n<origin rpy=\"0 0 0\" xyz=\"0 0.001 0\"\/>\\n<geometry>\\n<box size=\"0.030 0.009 0.020\"\/>\\n<\/geometry>\\n<\/collision>\\n<inertial>\\n<origin xyz=\"0 0 0\"\/>\\n<mass value=\"0.005\"\/>\\n<inertia ixx=\"0.001\" ixy=\"0.0\" ixz=\"0.0\" iyy=\"0.001\" iyz=\"0.0\" izz=\"0.001\"\/>\\n<\/inertial>\\n<\/link>\\n<joint name=\"imu_joint\" type=\"fixed\">\\n<parent link=\"base_link\"\/>\\n<child link=\"imu_link\"\/>\\n<origin rpy=\"0 0 0\" xyz=\"0.0 0 0.068\"\/>\\n<\/joint>\\n<link name=\"imu_link\"\/>\\n<joint name=\"scan_joint\" type=\"fixed\">\\n<parent link=\"base_link\"\/>\\n<child link=\"base_scan\"\/>\\n<origin rpy=\"0 0 0\" xyz=\"-0.064 0 0.122\"\/>\\n<\/joint>\\n<link name=\"base_scan\">\\n<visual>\\n<origin rpy=\"0 0 0\" xyz=\"0 0 0.0\"\/>\\n<geometry>\\n<mesh filename=\"package:\/\/turtlebot3_description\/meshes\/sensors\/lds.stl\" scale=\"0.001 0.001 0.001\"\/>\\n<\/geometry>\\n<material name=\"dark\"\/>\\n<\/visual>\\n<collision>\\n<origin rpy=\"0 0 0\" xyz=\"0.015 0 -0.0065\"\/>\\n<geometry>\\n<cylinder length=\"0.0315\" radius=\"0.055\"\/>\\n<\/geometry>\\n<\/collision>\\n<inertial>\\n<mass value=\"0.114\"\/>\\n<origin xyz=\"0 0 0\"\/>\\n<inertia ixx=\"0.001\" ixy=\"0.0\" ixz=\"0.0\" iyy=\"0.001\" iyz=\"0.0\" izz=\"0.001\"\/>\\n<\/inertial>\\n<\/link>\\n<joint name=\"camera_joint\" type=\"fixed\">\\n<origin rpy=\"0 0 0\" xyz=\"0.073 -0.011 0.084\"\/>\\n<parent link=\"base_link\"\/>\\n<child link=\"camera_link\"\/>\\n<\/joint>\\n<link name=\"camera_link\">\\n<collision>\\n<origin rpy=\"0 0 0\" xyz=\"0.005 0.011 0.013\"\/>\\n<geometry>\\n<box size=\"0.015 0.030 0.027\"\/>\\n<\/geometry>\\n<\/collision>\\n<\/link>\\n<joint name=\"camera_rgb_joint\" type=\"fixed\">\\n<origin rpy=\"0 0 0\" xyz=\"0.003 0.011 0.009\"\/>\\n<parent link=\"camera_link\"\/>\\n<child link=\"camera_rgb_frame\"\/>\\n<\/joint>\\n<link name=\"camera_rgb_frame\"\/>\\n<joint name=\"camera_rgb_optical_joint\" type=\"fixed\">\\n<origin rpy=\"-1.57 0 -1.57\" xyz=\"0 0 0\"\/>\\n<parent link=\"camera_rgb_frame\"\/>\\n<child link=\"camera_rgb_optical_frame\"\/>\\n<\/joint>\\n<link name=\"camera_rgb_optical_frame\"\/>\\n<\/robot>\\n\n\nRUN ON REMOTE PC\n$ export GAZEBO_MODEL_PATH=\"$GAZEBO_MODEL_PATH:$HOME\/model_folder\/\"\n","truncated":false}}
%---
%[output:2f5a714c]
%   data: {"dataType":"text","outputData":{"text":"average Hz is 76.88\n","truncated":false}}
%---
%[output:6506cd44]
%   data: {"dataType":"matrix","outputData":{"columns":6,"name":"AoA_error_all","rows":6,"type":"double","value":[["NaN","NaN","NaN","NaN","NaN","NaN"],["NaN","NaN","NaN","NaN","NaN","NaN"],["NaN","NaN","NaN","NaN","NaN","NaN"],["-2.5063","-7.8818","10.6876","NaN","50.7063","-1.1956"],["4.0830","-6.5501","-0.6207","-11.9678","NaN","-3.0059"],["NaN","NaN","NaN","NaN","NaN","NaN"]]}}
%---
%[output:8b30acd6]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"p_error","rows":2,"type":"double","value":[["1.4179"],["0.6454"]]}}
%---
%[output:2a00f09e]
%   data: {"dataType":"warning","outputData":{"text":"Warning: ros.Subscriber object cannot be saved to a MAT-file."}}
%---
%[output:1d765fac]
%   data: {"dataType":"warning","outputData":{"text":"Warning: ros.Publisher object cannot be saved to a MAT-file."}}
%---
%[output:45780115]
%   data: {"dataType":"warning","outputData":{"text":"Warning: ros.Subscriber object cannot be saved to a MAT-file."}}
%---
%[output:087a9b58]
%   data: {"dataType":"warning","outputData":{"text":"Warning: ros.Publisher object cannot be saved to a MAT-file."}}
%---
%[output:24cdb014]
%   data: {"dataType":"warning","outputData":{"text":"Warning: ros.Subscriber object cannot be saved to a MAT-file."}}
%---
%[output:7f65dd63]
%   data: {"dataType":"warning","outputData":{"text":"Warning: ros.Publisher object cannot be saved to a MAT-file."}}
%---
%[output:5bd43b8f]
%   data: {"dataType":"warning","outputData":{"text":"Warning: ros.Subscriber object cannot be saved to a MAT-file."}}
%---
%[output:50e3bec0]
%   data: {"dataType":"warning","outputData":{"text":"Warning: ros.Publisher object cannot be saved to a MAT-file."}}
%---
%[output:49639d24]
%   data: {"dataType":"warning","outputData":{"text":"Warning: ros.Subscriber object cannot be saved to a MAT-file."}}
%---
%[output:9a23175a]
%   data: {"dataType":"warning","outputData":{"text":"Warning: ros.Publisher object cannot be saved to a MAT-file."}}
%---
%[output:154a342c]
%   data: {"dataType":"warning","outputData":{"text":"Warning: ros.Subscriber object cannot be saved to a MAT-file."}}
%---
%[output:74560cc9]
%   data: {"dataType":"warning","outputData":{"text":"Warning: ros.Publisher object cannot be saved to a MAT-file."}}
%---
%[output:79566479]
%   data: {"dataType":"warning","outputData":{"text":"Warning: ros.Subscriber object cannot be saved to a MAT-file."}}
%---
%[output:84855855]
%   data: {"dataType":"warning","outputData":{"text":"Warning: ros.Publisher object cannot be saved to a MAT-file."}}
%---
%[output:052e1cbe]
%   data: {"dataType":"warning","outputData":{"text":"Warning: ros.Subscriber object cannot be saved to a MAT-file."}}
%---
%[output:16536903]
%   data: {"dataType":"warning","outputData":{"text":"Warning: ros.Publisher object cannot be saved to a MAT-file."}}
%---
%[output:271bf3e1]
%   data: {"dataType":"warning","outputData":{"text":"Warning: ros.Subscriber object cannot be saved to a MAT-file."}}
%---
%[output:72ab4671]
%   data: {"dataType":"warning","outputData":{"text":"Warning: ros.Publisher object cannot be saved to a MAT-file."}}
%---
%[output:922ebbb3]
%   data: {"dataType":"warning","outputData":{"text":"Warning: ros.Subscriber object cannot be saved to a MAT-file."}}
%---
%[output:3515b8cc]
%   data: {"dataType":"warning","outputData":{"text":"Warning: ros.Publisher object cannot be saved to a MAT-file."}}
%---
%[output:9445f046]
%   data: {"dataType":"warning","outputData":{"text":"Warning: ros.Subscriber object cannot be saved to a MAT-file."}}
%---
%[output:7618ea7b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: ros.Publisher object cannot be saved to a MAT-file."}}
%---
%[output:588d1f11]
%   data: {"dataType":"warning","outputData":{"text":"Warning: ros.Subscriber object cannot be saved to a MAT-file."}}
%---
%[output:9b272d43]
%   data: {"dataType":"warning","outputData":{"text":"Warning: ros.Publisher object cannot be saved to a MAT-file."}}
%---
%[output:7f10862f]
%   data: {"dataType":"text","outputData":{"text":"run 1, average Hz is 69.69\nrun 2, average Hz is 64.10\nrun 3, average Hz is 65.33\nrun 4, average Hz is 62.22\nrun 5, average Hz is 61.51\nrun 6, average Hz is 62.25\nrun 7, average Hz is 63.12\nrun 8, average Hz is 62.44\nrun 9, average Hz is 63.69\nrun 10, average Hz is 64.01\nrun 11, average Hz is 62.67\nrun 12, average Hz is 62.56\nrun 13, average Hz is 62.00\nrun 14, average Hz is 62.67\nrun 15, average Hz is 62.22\nrun 16, average Hz is 62.81\nrun 17, average Hz is 63.87\nrun 18, average Hz is 65.21\nrun 19, average Hz is 61.78\nrun 20, average Hz is 62.42\nrun 21, average Hz is 63.72\nrun 22, average Hz is 62.36\nrun 23, average Hz is 63.15\nrun 24, average Hz is 63.06\nrun 25, average Hz is 61.89\nrun 26, average Hz is 64.01\nrun 27, average Hz is 62.06\nrun 28, average Hz is 52.24\nrun 29, average Hz is 53.46\nrun 30, average Hz is 57.17\nrun 31, average Hz is 61.43\nrun 32, average Hz is 60.14\nrun 33, average Hz is 72.35\nrun 34, average Hz is 62.36\nrun 35, average Hz is 66.57\nrun 36, average Hz is 62.95\nrun 37, average Hz is 48.23\nrun 38, average Hz is 55.38\nrun 39, average Hz is 57.26\nrun 40, average Hz is 63.67\nrun 41, average Hz is 60.90\nrun 42, average Hz is 61.11\nrun 43, average Hz is 69.83\nrun 44, average Hz is 66.01\nrun 45, average Hz is 52.30\nrun 46, average Hz is 50.56\nrun 47, average Hz is 61.92\nrun 48, average Hz is 62.44\nrun 49, average Hz is 52.95\nrun 50, average Hz is 58.87\nrun 51, average Hz is 54.90\nrun 52, average Hz is 49.73\nrun 53, average Hz is 47.14\nrun 54, average Hz is 61.46\nrun 55, average Hz is 54.88\nrun 56, average Hz is 58.09\nrun 57, average Hz is 45.02\nrun 58, average Hz is 57.54\nrun 59, average Hz is 50.29\nrun 60, average Hz is 61.78\nrun 61, average Hz is 56.29\nrun 62, average Hz is 55.42\nrun 63, average Hz is 61.03\nrun 64, average Hz is 58.46\nrun 65, average Hz is 54.35\nrun 66, average Hz is 55.49\nrun 67, average Hz is 58.14\nrun 68, average Hz is 54.35\nrun 69, average Hz is 55.23\nrun 70, average Hz is 35.58\nrun 71, average Hz is 41.09\nrun 72, average Hz is 33.13\n","truncated":false}}
%---
%[output:5709c9db]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAWkAAABMCAIAAAAgD9EoAAAAB3RJTUUH6QcIFSAwI7thgAAACE9JREFUeJzt3c1PE80DB\/CZ1tLyYh4otpqWFkkQKSHQkHAwakKkQYKIxrMHjTefi1fjyYP+HXrw6ktMEI2JjcFIolEKBW2JWEpbi\/T9Zbdv2\/kdti3rCz6\/Z34gsb\/v51B3Z2dmZzf222GBgXZ1dREAgH9JtdcDAIA\/ErIDAHggOwCAB7IDAHggOwCAB7IDAHggOwCAB7IDAHggOwCAB7IDAHggOwCAB7IDAHggOwCAB7IDAHggOwCAB7IDAHggOwCAB7IDAHggOwCAB7IDAHggOwCAB7IDAHjs2+sBwI5pbm4+fvx4KBRyu91Go7GhoWFjY6NYLP60cn9\/\/9GjR\/ft2\/oPIAjCwsLCxsaG3W63WCwqlSoejz979kw+2tTUZLPZcrncyspKoVAghPT29lqt1rdv38ZiMUJIR0fH4OBgS0sLIcTj8czPzxNCWlpaBgYGOjo6KKWRSOT169eCIOz2fYDfQ93W1rbXY4CdodFojEZjOp3W6XRnz55taWnx+\/3bZUdbW9v+\/fvL5XKpVNJqtWNjYyaT6dOnTz09PZOTk\/F4XK1WT0xMZDIZn89HCLFYLJOTk4Ig+Hw+SZL6+\/svX75st9tdLlc0GjUajefOnRsaGtrY2Ojr6zt27JjP50ulUsPDw+fPny+Xyzqd7uTJk8VicXV1lTH2W+8L7A7MO+qHIAgvX74khNjtdpvNlslkflHZ4\/F4PB5CiFqtdjgchULh1atX6XR6bGzM7\/c\/fvxYFEWz2Xz69On3798nEgl5IhMMBguFwvj4+OTk5MGDB+PxuNxbT0+PzWZ78uTJixcvjEbjzZs3x8fH7927d+LEiUgkcv\/+\/XQ6fenSJYfD4XK5gsHgb7gbsNuQHfVDr9dfuXIllUrp9frDhw8bDIaGhoa1tbXR0dF4PG4ymVwu18OHD1OplLKV2WyemJjw+XxPnz4tFot3794VRTGZTJbL5Wg0euTIEYPBIIqi0WjM5\/OZTMZqtY6Oji4tLfn9\/u7ubkJIY2Oj2WyWJGltbS2fz6+vr7tcrsOHD5vN5kOHDs3NzYVCIUmSFhYWbDabyWRCdtQHPCutHxqNprOzs1gsOp3OQCDgdDqnp6d1Ot3AwEB7e\/vt27cfPHjw42TE4XDo9fq5ubl0Op3L5YLBYCwWK5fLNpttZGTE4\/F8\/vy5ra3NarV++fIlFouFQqFbt27duXMnnU7LPTQ2Nh44cEAQhFwuJ5cIgqDX63t6etRqdSKRkCSJEJLP5zUajcVi+Z33BHYP5h31plQqiaJYKpUEQchms4wxSqnb7Y5Goz+tPzQ05PV6nU5nrcRgMFy7ds1ut+dyuZmZmVKp1Nraqtfr5+fn5bxIJBLKHiilKpWKMaZ8kEEplR\/Efvd0Q61W79y1wl7CvKP+FQoF+VshPxoeHjYYDG63W1m4ubl548aNM2fOPHr06Pr16w6Ho7m5mVL63Rc7NYyxcrlMKaWUKgtLpRIhRFlICJHnIFAHkB3\/1wYHB4vF4vLysrzb1NRktVrb29tVKhUhxOl0ZrPZ\/v5+m80WjUa3e04himIkEmlqatLpdLV+YrGY1+uVJKm1tVWea2i12mKxuL6+\/luuDHYdsqMOSZIkSVJDQ4McAb9gMpmSyeTXr1\/lXavVevXq1YsXL\/7111\/yrkqlyuVyZrM5FApt91WPKIrBYFCtVnd2dmq1WovFMjg46Pf7g8FgOBy2Wq0mk6m1tXVgYCCbzYZCoZ29WNgreN5Rh+LxeCwW6+vrCwQCGo1mu2rNzc3t7e3xeLz2rdZQKPTmzZuRkZGpqal4PO5wOBYXF5eXl0+dOhWJRH5xRq\/X++HDh9HRUb1e39vbm8lkZmZmEonE7Ozs1NTUhQsXSqVSd3f39PR0OBze4auFPYKfDasroih6vd719fVUKpXNZgVBWFtbC4fDS0tLtYCo0Wq1hUJhcXExEAjIJfl8PhgMJhIJrVar0WhWV1enp6eTyWQymfz48WM2m1U2lyQpEAisrKyIopjNZsPhcC6X02g0m5ubz58\/d7vdkiRFo9FYLKZSqURRfPfu3ezsrPwzqVAHaFdX116PAQD+PHjeAQA8kB0AwAPZAQA8kB0AwAPZAQA8kB0AwAPZAQA8kB0AwAPZAQA8kB0AwAPZAQA8kB0AwIPzd\/A1fz\/f2XHA7lFRQilVUaKihBKiolRFCaVERYmK0uoGoaRS55v68gaR6\/+8oYooOqk25FZmhDFSZqTMGCPyBmGMfb9BSJkxZeWtbcKqhb9qeHBj1hJ+sXO3+U+lXG7yX8H6HfWPEUIZI4QyQiitFFQOMUYpJfKKorRShxFCq43kY6yycCBjhNDqQcIqfTHKKFH0Kx+prVJKqwdqr78uJ4TVdph8TlJd83Rr7VPGGGGVqkzZmDDCqvUUa6UyZTv8fZidgOyof5QQQimhpLpy6NasoJIJtFJpa7PaqNKwVl3Ro1xVebz2DyXKk1S3la\/blzNGKKGMEEJYpSNGKJVjgNLqG59SKkcGrdRXZBJltJJz1XZb6UQpJfT7BZiBB5531L\/aB3ft01txqPImlSttbSoaKat\/845jlaIfPtsr0wW+1+opmeKctfMqh6Ksuu2849vB1i7sH+4Y\/DeQHfWvOoX42byD\/NO8QzHr2OpA0Q1VHFduyOUcr9VTKuc6tfP+ZMZEvx2ivFdpoBjvN4P8Hx7HQA3WDQMAHph3AAAPZAcA8EB2AAAPZAcA8EB2AAAPZAcA8EB2AAAPZAcA8EB2AAAPZAcA8EB2AAAPZAcA8EB2AAAPZAcA8EB2AAAPZAcA8EB2AAAPZAcA8EB2AAAPZAcA8EB2AAAPZAcA8EB2AACP\/wAEGsQzrVn4+AAAAABJRU5ErkJggg==","height":76,"width":360}}
%---
%[output:0d422864]
%   data: {"dataType":"error","outputData":{"errorType":"runtime","text":"Error using <a href=\"matlab:matlab.lang.internal.introspective.errorDocCallback('ros.Subscriber\/receive', 'C:\\Program Files\\MATLAB\\R2025a\\toolbox\\ros\\mlroscpp\\+ros\\Subscriber.m', 465)\" style=\"font-weight:bold\">ros.Subscriber\/receive<\/a> (<a href=\"matlab: opentoline('C:\\Program Files\\MATLAB\\R2025a\\toolbox\\ros\\mlroscpp\\+ros\\Subscriber.m',465,0)\">line 465<\/a>)\nThe function did not receive any data and timed out.\n\nError in <a href=\"matlab:matlab.lang.internal.introspective.errorDocCallback('MRS25>runExperiment', 'C:\\Users\\b.dijkstra\\OneDrive - University of Groningen\\Documenten\\Industrial Engineering & Management\\2.0 MSc Industrial Engineering & Management\\4.2 Research Project - WiFi-CSI-Sensing\\_ICRA25\\MATLAB\\MRS25.m', 954)\" style=\"font-weight:bold\">MRS25>runExperiment<\/a> (<a href=\"matlab: opentoline('C:\\Users\\b.dijkstra\\OneDrive - University of Groningen\\Documenten\\Industrial Engineering & Management\\2.0 MSc Industrial Engineering & Management\\4.2 Research Project - WiFi-CSI-Sensing\\_ICRA25\\MATLAB\\MRS25.m',954,0)\">line 954<\/a>)\n                msgSub = R(i).rosSub.receive(1);\n                ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^"}}
%---
%[output:93ac956b]
%   data: {"dataType":"textualVariable","outputData":{"name":"ans","value":"1.4077"}}
%---
%[output:0e6cbb9e]
%   data: {"dataType":"textualVariable","outputData":{"name":"ans","value":"1.9087"}}
%---
%[output:5002d5ef]
%   data: {"dataType":"textualVariable","outputData":{"name":"ans","value":"1.2224"}}
%---
%[output:0b87372c]
%   data: {"dataType":"textualVariable","outputData":{"name":"ans","value":"1.7043"}}
%---
%[output:526f36da]
%   data: {"dataType":"textualVariable","outputData":{"name":"ans","value":"0.9272"}}
%---
%[output:55315153]
%   data: {"dataType":"textualVariable","outputData":{"name":"ans","value":"1.1541"}}
%---
%[output:04a2a651]
%   data: {"dataType":"matrix","outputData":{"columns":6,"name":"ans","rows":1,"type":"double","value":[["5.1435","25.3023","25.6364","NaN","91.2969","0.3310"]]}}
%---
%[output:3ee7dd84]
%   data: {"dataType":"matrix","outputData":{"columns":6,"name":"ans","rows":1,"type":"double","value":[["7.4880","46.5393","65.9802","87.2456","NaN","7.2726"]]}}
%---
%[output:534dc215]
%   data: {"dataType":"matrix","outputData":{"columns":6,"name":"ans","rows":1,"type":"double","value":[["1.5060","21.9210","6.3577","NaN","97.2470","0.1897"]]}}
%---
%[output:86113513]
%   data: {"dataType":"matrix","outputData":{"columns":6,"name":"ans","rows":1,"type":"double","value":[["3.0762","28.6297","54.2630","88.7201","NaN","6.0280"]]}}
%---
%[output:4090efa9]
%   data: {"dataType":"matrix","outputData":{"columns":6,"name":"ans","rows":1,"type":"double","value":[["9.0753","22.0205","40.6512","NaN","60.5725","0.3599"]]}}
%---
%[output:48f83406]
%   data: {"dataType":"matrix","outputData":{"columns":6,"name":"ans","rows":1,"type":"double","value":[["21.6345","50.7126","54.9444","57.2754","NaN","15.4783"]]}}
%---
