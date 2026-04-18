function [success, sub, pub, msg, service] = rosSpawnClientService(RosVersion,client,xml,x,y,z,name,ns,master)
    
% predefine parameters
success=true;

switch lower(RosVersion)
    case "noetic"

        % Services for spawning
        service = rosmessage(client);
        
        % Set parameters
        service.ModelXml = xml;
        service.ModelName = name;
        service.RobotNamespace = ns;
        service.ReferenceFrame = 'world';
        service.InitialPose.Position.X = x;
        service.InitialPose.Position.Y = y;
        service.InitialPose.Position.Z = z;

        % Attempt to spawn
        response = call(client, service, 'Timeout', 10);

        % Check spawn success
        if ~response.Success
            success = false;
            if strcmp(response.StatusMessage,'SpawnModel: Failure - entity already exists.')
                warning(service.ModelName + " already exists, skipped");
            else
                showdetails(response)
                error(service.ModelName + " not spawned");
            end                
        end
        disp("Spawn success " + name)

        % Store subscriber and publisher
        sub = rossubscriber(strcat(ns, '/odom'),'nav_msgs/Odometry','BufferSize',3,'DataFormat','struct');
        pub = rospublisher(strcat(ns, '/cmd_vel'),'geometry_msgs/Twist','DataFormat','struct');
        msg = rosmessage(pub);


    case "jazzy"
        % ROS 2 + Gazebo Sim (gz) spawn script for ground robots
        % Tested for ROS 2 Jazzy and Gazebo Fortress/Harmonic

        % Services for spawning
        service = ros2message(client);
       
        % Set parameters
        service.entity_factory.sdf = xml;
        service.entity_factory.name = name;
        service.entity_factory.pose.position.x = x;
        service.entity_factory.pose.position.y = y;
        service.entity_factory.pose.position.z = z;
        service.entity_factory.pose.orientation.x = 0.0;
        service.entity_factory.pose.orientation.y = 0.0;
        service.entity_factory.pose.orientation.z = 0.0;
        service.entity_factory.pose.orientation.w = 1.0;
        service.entity_factory.allow_renaming = false; % don't allow gz to rename in case of conflict

        % Attempt to spawn
        try
            response = call(client, service, "Timeout", 15);
            disp("Spawn success " + name)
        catch E
            success = false;
            warning(E.identifier, "Failed to spawn: %s", E.message)
        end

        % Topics bridged from Gazebo via ros_gz_bridge
        odom_topic = "/model/" + name + "/odometry";
        cmdvel_topic = "/model/" + name + "/cmd_vel";
    
        % Create ROS 2 publishers and subscribers
        sub = ros2subscriber(master, odom_topic, "nav_msgs/Odometry");
        pub = ros2publisher(master, cmdvel_topic, "geometry_msgs/Twist");
        msg = ros2message(pub);
        
end

end

%[appendix]{"version":"1.0"}
%---
