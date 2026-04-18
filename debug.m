function [] = debug(param)

if param.ros
    switch lower(conf.RosVersion)
        case "noetic"
            disp("__Services__"); rosservice list
            disp("__Nodes_____"); rosnode list
            disp("__Topics____"); rostopic list
            disp("__Gazebo____"); rosnode('info','/gazebo')
        case "jazzy" 
            disp("__Services__"); ros2 service list
            disp("__Nodes_____"); ros2 node list
            disp("__Topics____"); ros2 topic list
    end
end

%[text] #### 
% Check on the nexus car information
if param.nexus
    if ~is3D(1)
        rostopic info /r1_ground/odom
        rostopic info /r1_ground/cmd_vel
    elseif is3D(1)
        rostopic info /r1_drone/odom
        rostopic info /r1_drone/cmd_vel
    else
        warning("no debugging for this robot type")
    end
    rosmsg info geometry_msgs/Twist
    rosmsg info nav_msgs/Odometry
end


end


%[appendix]{"version":"1.0"}
%---
