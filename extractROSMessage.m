function [pos, ori, simTime, isOK] = extractROSMessage(msgSub, RosVersion)
% helper function to obtain info from ros message in different ROS versions
isOK = true; % Initialize isOK to true
    switch lower(RosVersion)
        case 'noetic'
            pos = struct2cell(msgSub.Pose.Pose.Position)';
            ori = struct2cell(msgSub.Pose.Pose.Orientation)';
            simTime = double(msgSub.Header.Stamp.Sec)+double(msgSub.Header.Stamp.Nsec)*1e-9; % simulation time
        case 'jazzy'
            pos = struct2cell(msgSub.pose.pose.position)';
            ori = struct2cell(msgSub.pose.pose.orientation)';
            simTime = double(msgSub.header.stamp.sec)+double(msgSub.header.stamp.nanosec)*1e-9; % simulation time
        otherwise
            error("ROS version undefined")
    end

    if any(isnan(cell2mat([pos,ori])))
        isOK = false;
    end
end

%[appendix]{"version":"1.0"}
%---
