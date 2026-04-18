function [] = checkAndBuildROSMsgs(RosVersion, resetMsgs)
%% Generate ROS messages 

% Initialize resetMsgs if not provided
if nargin < 2
    resetMsgs = false;
end

% based on ros_gz_interfaces folder into matlab_msg_gen
switch lower(RosVersion)
    case 'jazzy'
        % Do full reset of custom messages
        if resetMsgs
            % Delete old messages folder
            if isfolder('matlab_msg_gen')
                rmdir("matlab_msg_gen/", 's')
            end
            % Refresh
            reg = ros.internal.CustomMessageRegistry.getInstance('ros2', true); 
            reg.refresh(true)
            disp("full reset of ros2 msgs complete")
        end
        % Obtain current list of accesible msgs
        ros2msgList = ros2("msg","list");
        % Check if ros messages are missing
        if ~any(contains(ros2msgList,'ros_gz_interfaces'))
            if ~isfolder('ros_gz_interfaces')
                error('Need to rebuild ROS2 messages from "ros_gz_interfaces" which does not exist in the current dir');
            end
            % Generate the messages
            warning("We are now going to generate ROS2 msgs, in case you run into issues make sure to check the documentation")
            ros2genmsg()
        end
    case 'noetic'
        warning("no code to check proper working of ROS1 msgs yet")
    otherwise
        warning("no code to check proper working of msgs yet on this ros version")
end
end

%[appendix]{"version":"1.0"}
%---
