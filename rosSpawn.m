function [rosSub,rosPub,msgPub,rosNS, spawn]=rosSpawn(xml_path,numR,posStart,platformWidth,RosVersion,master,worldName)

% Get the SDF/URDF file in text from the file
[xml_string, model_ext] = xml_parser(xml_path);


%% Define robot number of gz true pose IDs
% ROS2 messages co not maintain the header data from gz message, which
% means we have to work around this knowing the order of the links.
% We keep track of what we spawn and knwo how the list will generate to get
% the right index to take in MATLAB.
% N = num robots, P = num pedestals (0<=P<N)
%_____________________________________
% GZ pose list index based on Turtlebot waffle:
% (1)    ground plane
% (1*P)  pedestal
% (1*N)  robotN
% (1)    link
% (10*N) base_footprint, base_link, imu_link, base_scan, wheel_left_link, wheel_right_link, ...
%        caster_back_right_link, caster_back_left_link, camera_link, camera_rgb_frame, 
% (2)    visual, sunVisual, 
% (4*N)  base_visual, lidar_sensor_visual, wheel_left_visual, wheel_right_visual
% (1)    sun
%_____________________________________
% configure the function below to aim for the right subsciber, some examples:
%spawnIndexFunc = @(N, P) 1 + (1*P) + (1*N) + 1 + 10 * (N-1) + 1; % base_footprint
spawnIndexFunc = @(N, P) 1 + (1*P) + (1*N); % robotN
N = 0; P = 0;


% Test if gazebo node is reachable
switch lower(RosVersion)
    case "noetic"
        try [~]=rosnode('info','/gazebo'); 
        catch; error("Gazebo not reachable"); 
        end
    case "jazzy"

end

% Pre-define data holders
rosSub = cell(1, numR);
rosPub = cell(1, numR);
msgPub = cell(1, numR);
rosNS = cell(1, numR);
spawn = cell(1, numR);

% Create spawing client for robots and pedestals
switch lower(RosVersion)
    case "noetic"
        % Spawn client robots
        switch string(model_ext)
            case "sdf"
                track_spawn_client = rossvcclient("/gazebo/spawn_sdf_model",'IsPersistent',false); %,'DataFormat','struct',"Timeout",10
            case "urdf"
                track_spawn_client = rossvcclient("/gazebo/spawn_urdf_model",'IsPersistent',false);
            otherwise
                error("spawning ground robot extension not supported: " + model_ext)
        end
        
        % Spawn client pedestals
        box_spawn_client = rossvcclient("/gazebo/spawn_urdf_model",'IsPersistent',false);

    case "jazzy"
        % Spawn client robots
        track_spawn_client = ros2svcclient(master, sprintf("/world/%s/create",worldName), "ros_gz_interfaces/SpawnEntity");

        % Spawn client pedestals
        box_spawn_client = ros2svcclient(master, sprintf("/world/%s/create",worldName), "ros_gz_interfaces/SpawnEntity");
end


% Spawn each robot individually
for i=1:numR

    %%% Spawn pedestal
    box_height = posStart(i,3);
    box_height = box_height-0.01; % collision space of robot.
    box_width = platformWidth(i,:);
    
    if box_height>0.05
        
        % Spawn model parameters
        xml = generate_box_urdf(box_height,box_width);
        name = ['pedestal_robot', num2str(i)];
        ns = ['/', name]; % namespace of robot for ROS1

        x = posStart(i,1);
        y = posStart(i,2);
        z = box_height / 2;  % Center at half-height

        % Call spawn function with given inputs
        rosSpawnClientService(RosVersion,box_spawn_client,xml,x,y,z,name,ns,master);
        
        % Keep track of spawned pedestals for true pose
        P = P + 1;
    end
    
    %%% Spawn robot
    % Spawn model parameters
    name = sprintf('robot%i', i);
    ns = strcat("/", name); % namespace of robot for ROS1

    x = posStart(i,1); % set spawn location X
    y = posStart(i,2); % set spawn location Y
    z = posStart(i,3); % set spawn location Z

    % XML SDF/URDF robot file
    switch lower(RosVersion)
        case "noetic"
            % Namespace is defined using ns parameter
            xml = xml_string;
        case "jazzy"
            % Adjust SDF file to iterate robot namespace manually
            xml = strrep(xml_string, '<topic>cmd_vel</topic>', sprintf('<topic>/model/%s/cmd_vel</topic>', name));
            xml = strrep(xml,'<odom_topic>odom</odom_topic>', sprintf('<odom_topic>/model/%s/odometry</odom_topic>', name));
            xml = strrep(xml,'<tf_topic>tf</tf_topic>', sprintf('<tf_topic>/model/%s/tf</tf_topic>', name));
    end
    
    % Call spawn function with given inputs
    [~, sub, pub, msg, ~] = rosSpawnClientService(RosVersion,track_spawn_client,xml,x,y,z,name,ns,master);   
    
    % Keep track of spawned robots for true pose
    N = N + 1;

    % Store subscriber, publisher, pub message and namespace per robot
    rosSub{i} = sub;
    rosPub{i} = pub;
    msgPub{i} = msg;
    rosNS{i} = ns;
    spawn{i} = spawnIndexFunc(N,P);
end

end

%[appendix]{"version":"1.0"}
%---
