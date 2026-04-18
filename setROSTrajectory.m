function msgPub = setROSTrajectory(msgPub, vx, vy, rz, RosVersion)
% Set the trajectory publish message for any ROS version

    switch lower(RosVersion)
        case 'noetic'
            msgPub.Linear.X = vx;
            msgPub.Linear.Y = vy;
            msgPub.Angular.Z = rz;
        case 'jazzy'
            msgPub.linear.x = vx;
            msgPub.linear.y = vy;
            msgPub.angular.z = rz;
        otherwise
            error("ROS version undefined")
    end
end