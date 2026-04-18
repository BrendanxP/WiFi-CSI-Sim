function pids = runSystemCmd(cmd)
    [~, rosLdPathRaw] = system('bash -lc ''source /opt/ros/jazzy/setup.bash && source ~/turtlebot3_ws/install/setup.bash && echo "$LD_LIBRARY_PATH"''');
    rosLdPath = strtrim(rosLdPathRaw);
    %rosLdPath = strtrim(fileread('/tmp/ros2_ldpath.txt')); file no longer there

    % Manual cause automatic broke, get using echo $LD_LIBRARY_PATH
    rosLdPath = "/home/brendan/turtlebot3_ws/install/turtlebot3_msgs/lib:/home/brendan/turtlebot3_ws/install/dynamixel_sdk_custom_interfaces/lib:/home/brendan/turtlebot3_ws/install/dynamixel_sdk/lib:/opt/ros/jazzy/opt/gz_sim_vendor/lib:/opt/ros/jazzy/opt/gz_sensors_vendor/lib:/opt/ros/jazzy/opt/gz_physics_vendor/lib:/opt/ros/jazzy/opt/sdformat_vendor/lib:/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/jazzy/opt/gz_gui_vendor/lib:/opt/ros/jazzy/opt/gz_transport_vendor/lib:/opt/ros/jazzy/opt/gz_rendering_vendor/lib:/opt/ros/jazzy/opt/gz_plugin_vendor/lib:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/lib:/opt/ros/jazzy/opt/gz_msgs_vendor/lib:/opt/ros/jazzy/opt/gz_common_vendor/lib:/opt/ros/jazzy/opt/gz_math_vendor/lib:/opt/ros/jazzy/opt/gz_utils_vendor/lib:/opt/ros/jazzy/opt/gz_tools_vendor/lib:/opt/ros/jazzy/opt/gz_ogre_next_vendor/lib:/opt/ros/jazzy/opt/gz_dartsim_vendor/lib:/opt/ros/jazzy/opt/gz_cmake_vendor/lib:/opt/ros/jazzy/lib";

    sysLibStdCpp = "/usr/lib/x86_64-linux-gnu/libstdc++.so.6";
    
    envLines = [
        "export TURTLEBOT3_MODEL=waffle"
        "export GZ_VERSION=garden"
        "export GZ_SIM_RESOURCE_PATH=/home/brendan/turtlebot3_ws/src/turtlebot3/"
        "export GZ_PLUGIN_PATH=/opt/ros/jazzy/opt/gz_sim_vendor/lib/gz-sim-8/plugins/:/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins/"
        "source /opt/ros/jazzy/setup.bash"
        "source ~/turtlebot3_ws/install/setup.bash"
    ];
    envCmd = strjoin(envLines, " && ");
    
    % For ros2 launch: capture its output to parse the actual child PID
    if contains(cmd, "ros2 launch")
        % Run ros2 launch with output captured, then background the children
        fullCmdInner = envCmd + " && " + cmd + " & sleep 1 && jobs -p";
        wrapped = "env LD_PRELOAD=" + sysLibStdCpp + ...  
                  " LD_LIBRARY_PATH=""" + rosLdPath + """" + ...
                  " bash -lc '" + fullCmdInner + "'";
        
        [status, output] = system(wrapped);
        if status ~= 0
            error("Failed to start command: %s", output);
        end
        
        % Parse 'jobs -p' output for background job PIDs
        lines = strsplit(output, '\n');
    
        % Initialize PIDs
        bashPid = NaN;
        launchPid = NaN;
        gazeboPid = NaN;
        
        % Parse all 3 PIDs
        for i = 1:length(lines)
            line = strtrim(lines{i});
            
            % 1) Bash wrapper PID: from 'echo $$' (current shell PID)
            if ~isnan(str2double(line)) && length(line) > 2
                if isnan(bashPid)  % Take first pure number as bash PID
                    bashPid = str2double(line);
                elseif isnan(launchPid)
                    launchPid = str2double(line);  % Next is jobs -p (ros2)
                end
            end
            
            % 2) Gazebo PID: [INFO] [gazebo-1]: process started with pid [NNNNN]
            gazeboMatch = regexp(line, '\[(\d+)\]', 'tokens');
            if ~isempty(gazeboMatch)
                gazeboPid = str2double(gazeboMatch{1}{1});
            end
            
            % 3) Launch session PID: after last '-' in log path
            %    /home/.../brendan-Framework-Ryzen-AI-300-18402
            dashMatch = regexp(line, '([^/-]+)-(\d+)$', 'tokens');
            if ~isempty(dashMatch)
                candidatePid = str2double(dashMatch{1}{end});
                if ~isnan(candidatePid) && candidatePid > 1000
                    launchPid = candidatePid;  % Override with log path PID
                end
            end
        end
        
        % Combine into array: [bash_pid, ros2_launch_pid, gazebo_pid]
        pids = [bashPid, launchPid, gazeboPid];
        pids(isnan(pids)) = [];  % Remove missing PIDs
        
    else
        % For non-ros2 commands, use pgrep fallback
        fullCmdInner = envCmd + " && (" + cmd + ") & echo $!";
        wrapped = "env LD_PRELOAD=" + sysLibStdCpp + ...
                  " LD_LIBRARY_PATH=""" + rosLdPath + """" + ...
                  " bash -lc '" + fullCmdInner + "'";
        
        [status, bashPidStr] = system(wrapped);
        if status ~= 0
            error("Failed to start command: %s", bashPidStr);
        end
        
        pids = str2double(strtrim(bashPidStr));
        pause(0.5);
        
        % Try pgrep as backup
        [~, grepOut] = system(sprintf('pgrep -f "%s"', cmd(1:min(20,end))));
        childPids = str2double(strsplit(strtrim(grepOut), '\n'));
        childPids = childPids(childPids > 0 & childPids ~= pids);
        if ~isempty(childPids)
            pids = childPids(1);
        end
    end
end
