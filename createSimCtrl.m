function simCtrl = createSimCtrl(master, worldName, RosVersion)

% Define world controls and true position subscriber
switch lower(RosVersion)
    case 'noetic' 
        simCtrl.pause = rossvcclient('/gazebo/pause_physics'); % Pause and unpause e.g. to do heavy calculations during run
        simCtrl.unpause = rossvcclient('/gazebo/unpause_physics');
        simCtrl.resetWorld = rossvcclient('/gazebo/reset_world'); % resets robots, not time
        simCtrl.resetSim = rossvcclient('/gazebo/reset_simulation'); % reset robots and time
        simCtrl.physicsClient = rossvcclient('/gazebo/set_physics_properties'); % Set Gazebo simulation time to slowdown for reaching desired rate
    case 'jazzy'
        % Make control client
        simCtrl.client = ros2svcclient(master, sprintf("/world/%s/control",worldName), "ros_gz_interfaces/ControlWorld");
        
        % pause sim
        simCtrl.pause = ros2message(simCtrl.client);
        simCtrl.pause.world_control.pause = true;  % pause the simulation
        %call(simCtrl.client, simCtrl.pauseSim);

        % unpause sim
        simCtrl.unpause = ros2message(simCtrl.client);
        simCtrl.unpause.world_control.pause = false;  % pause the simulation
        %call(simCtrl.client, simCtrl.unpauseSim);

        % set fixed seed (is bugged)
        simCtrl.setSeed = ros2message(simCtrl.client);
        simCtrl.setSeed.world_control.seed = uint32(1); 
        %call(simCtrl.client, simCtrl.setSeed);
        
        % reset sim time
        simCtrl.resetSim = ros2message(simCtrl.client);
        simCtrl.resetSim.world_control.reset.model_only = true;   % should only reset time but robots also disappear...
        %call(simCtrl.client, simCtrl.resetSim)
        
        % reset world
        simCtrl.resetWorld = ros2message(simCtrl.client);
        simCtrl.resetWorld.world_control.reset.all = true;   % reset robots and time
        %call(simCtrl.client, simCtrl.resetWorld) use this to reset world
end

end

%[appendix]{"version":"1.0"}
%---
