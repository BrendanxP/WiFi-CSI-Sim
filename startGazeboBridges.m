function [fullCmd, pid] = startGazeboBridges(numRobots, worldName,manualTerminal)
% startGazeboBridges Start ros_gz_bridge parameter_bridge for N robots.
% pid = startGazeboBridges(N) launches a ros_gz_bridge process that
% bridges /clock and /robot<i>/{cmd_vel,odom,tf} for i = 1..N.
% pid = startGazeboBridges(N, worldName) uses the specified world name.
% If worldName is not provided, defaults to "default".
%
% Returns the PID of the launched process (Linux).

    if nargin < 1
        error('Provide numRobots as input.');
    end
    if nargin < 2
        worldName = "default";
    end
    if nargin < 3
        manualTerminal = 1;
    end

    % Base command
    cmd = "ros2 run ros_gz_bridge parameter_bridge";

    % World poses (all robots true pos/ori, like /gazebo/model_states)
    cmd = cmd + sprintf(" /world/%s/pose/info@geometry_msgs/msg/PoseArray@gz.msgs.Pose_V", worldName);
    % TF for all poses (names preserved: robot1/base_link -> map)
    %cmd = cmd + sprintf(" /world/%s/pose/info[tf2_msgs/msg/TFMessage@gz.msgs.Pose_V", worldName);

    
    % World clock
    cmd = cmd + " /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock";

    % Per-robot topics
    for i = 1:numRobots
        ns = "/model/robot" + i;

        % cmd_vel
        cmd = cmd + " " + ns + "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist";

        % odom
        cmd = cmd + " " + ns + "/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry";

        % tf
        cmd = cmd + " " + ns + "/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V";
    end

    % Spawn / control services (optional, if not launched separately)
    % Add world services (create, remove, etc.)
    cmd = cmd + sprintf(" /world/%s/create@ros_gz_interfaces/srv/SpawnEntity", worldName);
    cmd = cmd + sprintf(" /world/%s/control@ros_gz_interfaces/srv/ControlWorld", worldName);

    % Finalize full command
    fullCmd = sprintf('%s & echo $!', cmd);

    if manualTerminal
        link_text = 'Bridge to control the world and spawn robots)';
        createCopyLink(cmd, link_text); % Click to copy the command to bridge ROS2 with GZ
        pid = [];
    else
        pid = runSystemCmd(cmd);
        pause(2);
    end

end


%[appendix]{"version":"1.0"}
%---
