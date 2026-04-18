function [odomT,poseT,csiT] = preExperiment(simCtrl,xml_path,numR,posStart,RosVersion,master,worldName)

% Predefine odom and CSI data timetables
odomT = timetable('Size',[0 7], 'RowTimes',datetime.empty(0,1), ...
    'VariableTypes',{'double','double','double','double','double','double','double'}, ...
    'VariableNames', {'pos_x','pos_y','pos_z','quat_x','quat_y','quat_z','quat_w'});
poseT = timetable('Size',[0 7], 'RowTimes',datetime.empty(0,1), ...
    'VariableTypes',{'double','double','double','double','double','double','double'}, ...
    'VariableNames', {'pos_x','pos_y','pos_z','quat_x','quat_y','quat_z','quat_w'});
csiT = timetable('Size',[0 3], 'RowTimes',datetime.empty(0,1), ...
    'VariableTypes',{'double','string','cell'}, ...
    'VariableNames',{'itr','MAC','CSI'});


% Reset robot positions and time
switch lower(RosVersion)
    case 'noetic'
        call(simCtrl.resetSimulation); % resets robots to start
    case 'jazzy'
        %call(simCtrl.client, simCtrl.resetSim); % also removes robots
        %rosSpawn(xml_path,numR,cell2mat(posStart),RosVersion,master,worldName);
        call(simCtrl.client, simCtrl.unpause); % start!
end

end

%[appendix]{"version":"1.0"}
%---
