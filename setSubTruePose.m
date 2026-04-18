function sub = setSubTruePose(master, worldName, RosVersion)
% True pose subscriber
    switch lower(RosVersion)
        case 'noetic' 
            sub = [];
            warning("no true pose tested")
    
        case 'jazzy'
            sub = ros2subscriber(master,sprintf('/world/%s/pose/info',worldName), 'geometry_msgs/PoseArray');
    end

%[appendix]{"version":"1.0"}
%---
