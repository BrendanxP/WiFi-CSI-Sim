function restartSim(confirmed, pids)
% Input true to bypass confirmation step

    arguments
        confirmed(1,1) logical = 0;
        pids = [];
    end

    if ~confirmed
        % Prompt for confirmation
        confirm = input('Are you sure you want to shutdown ROS, clear the workspace, and close all figures? [y/n]: ', 's');
        if ~(strcmpi(confirm, 'y') || strcmpi(confirm, 'yes'))
            disp("restart cancelled")
            return;
        end
    end

    % Kill running programs
    killSystemCmd(pids)
    killROS();
    killGZ();

    % Reset workspace
    evalin("base","clearvars -except -regexp mc_*");
    evalin("base","close all");
    evalin("base","clc");
end