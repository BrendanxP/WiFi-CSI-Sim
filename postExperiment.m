function [] = postExperiment(simCtrl,RosVersion)

% Pause sim to do calculations
switch lower(RosVersion)
    case 'noetic'
        call(simCtrl.pause);
    case 'jazzy'
        call(simCtrl.client, simCtrl.pause); 
end

end


%[appendix]{"version":"1.0"}
%---
