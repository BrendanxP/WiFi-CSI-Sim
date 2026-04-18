function rate = setROSRate(master, ros_rate, RosVersion)
% Obtain rate object for any ROS version and set sim parameters where
% needed.

    switch lower(RosVersion)
        case 'noetic' 
            % Set rate parameters
            rate = rosrate(ros_rate); % rate based on simulation time
            rate.OverrunAction='slip'; % 'slip' immediately execute loop again when late, otherwise 'drop'
            
            % Get physics client
            getPhysicsClient = rossvcclient('/gazebo/get_physics_properties');
            simPhysics = call(getPhysicsClient); % Get current physics properties
    
            % Configure new physics properties to get best performance
            simPhysics.MaxUpdateRate = 1000;    % Default: 1000 [Hz] max internal update rate
            simPhysics.TimeStep = 1e-3;         % Default 1e-3 [s] timestep
            
            % Apply new settings
            response = call(physicsClient, simPhysics);
    
            if ~response.Success
                p_error('Failed to adjust simulation speed');
            end
    
            % Get info on ros simulation timing
            rosparameters.sim_time = rosparam("get", "/use_sim_time");
            rosparameters.time_step = rosparam("get", "/gazebo/time_step");
    
        case 'jazzy'
            rate = ros2rate(master,ros_rate);
    end
end

%[appendix]{"version":"1.0"}
%---
