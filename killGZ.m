function killGZ(force)
    % MATLAB version of your bashrc killgz() function
    % Kills all Gazebo Sim (gz) and related ROS processes

    arguments
        force(1,1) logical = 0;
    end
    
    patterns = {
        'gz sim',           % gz sim-server
        'ros_gz_bridge',    % ROS-Gazebo bridge
        'gz launch',        % gz launch files
        'gz transport',     % Gazebo transport nodes
        'gz.*server',       % Any gz server
        'gz'                % Generic gz processes
        'ros2 launch.*gz'   % Any lingering ROS launches
    };
    
    for i = 1:length(patterns)
        pattern = patterns{i};
        %fprintf('  Killing: %s\n', pattern);
        
        if ~force
            % SIGTERM terminate
            system(sprintf('pkill -f "%s" 2>/dev/null', pattern));
        else
            % Force kill
            system(sprintf('pkill -9 -f "%s" 2>/dev/null', pattern));
        end

        pause(0.1);
    end
end