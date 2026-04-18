function [master, gzPids] = initROS(conf)
    

    
    % Start ROS services in MATLAB
    switch lower(conf.RosVersion)
        case "noetic"
            % Test connection
            CL = checkConnection(conf.LocalIP);
            CR = checkConnection(conf.RemoteIP);
            
            if ~(CL && CR)
                error("Local IP reachable: %s.\nRemote IP reachable: %s.",mat2str(CL),mat2str(CR))
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
            disp("rosrun gazebo_ros gazebo (empty world, connected to ROS, with GUI)")
            disp("roslaunch gazebo_ros empty_world.launch gui:=false (empty world, connected to ROS, no GUI)")

        case "jazzy"
            % Make connection and test with some basic info commands
            master = ros2node("/matlab");
            ros2 node list
            ros2 topic list
            
            % Set type of gz world to open
            switch conf.gzWorld
                case "empNoGUI"
                    % Command 1
                    cmd = 'ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="empty.sdf -s"';
                    link_text = 'ROS2 GZ world empty (no GUI)';
                case "empGUI"
                    % Command 2
                    cmd = 'ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="empty.sdf"';
                    link_text = 'ROS2 GZ world empty (w/ GUI)';
                case "testTurtle"
                    % Command 3
                    cmd = 'ros2 launch turtlebot3_gazebo empty_world.launch.py';
                    link_text = 'ROS2 GZ world with turtlebot3 for testing only (w/ GUI)';
                otherwise
                    warning("gz world option not defined")
                    cmd = "";
                    link_text = "undefined";
            end
            
            % Manual or automatic opening
            if conf.manualTerminals
                % Show copy-pastable command
                createCopyLink(cmd, link_text);
                gzPids = NaN;
            else
                % Execute in background
                gzPids = runSystemCmd(cmd);
            end

        otherwise
            error("Ros version not defined")
    end
end

%[appendix]{"version":"1.0"}
%---
