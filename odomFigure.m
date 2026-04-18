function hFig = odomFigure(odomT, poseT, posStart, range, i, plotVisible)
% ODOMFIGURE Plots odom & pose trajectories with color gradients over time.

% Check input
if ~istimetable(odomT) || ~istimetable(poseT)
    error('Inputs must be timetables.');
end
if ~all(ismember({'pos_x','pos_y'}, odomT.Properties.VariableNames))
    error('odomT must contain variables ''pos_x'' and ''pos_y''.');
end
if ~all(ismember({'pos_x','pos_y'}, poseT.Properties.VariableNames))
    error('poseT must contain variables ''pos_x'' and ''pos_y''.');
end
if ~exist("plotVisible","var")
    plotVisible = 'on'; % default to show
end

% Extract data
odom_x = odomT.pos_x + posStart(1);
odom_y = odomT.pos_y + posStart(2);
odom_t = odomT.Time;
pose_x = poseT.pos_x;
pose_y = poseT.pos_y;
pose_t = poseT.Time;

% Compute center
mid_x = (min([odom_x;pose_x]) + max([odom_x;pose_x])) / 2;
mid_y = (min([odom_y;pose_y]) + max([odom_y;pose_y])) / 2;

% Compute axis limits
xlim_centered = [mid_x - range/2, mid_x + range/2];
ylim_centered = [mid_y - range/2, mid_y + range/2];

% time gradient as z axis for plot
min_t = min([odom_t;pose_t]);
odom_z = seconds(odom_t - min_t);
pose_z = seconds(pose_t - min_t);

% Scale z to indices: odom 1-128 (blue-purple), pose 129-256 (yellow-red)
odom_z_norm = (odom_z - min(odom_z)) / (max(odom_z) - min(odom_z)) * 127 + 1;
pose_z_norm = (pose_z - min(pose_z)) / (max(pose_z) - min(pose_z)) * 127 + 128;

% Create figure
hFig = figure('visible',plotVisible,"Theme","Light");
hold on;

% create color ranges
blue_purple = [0 0 1; 0.5 0 0.5];
blue_purple = interp1(linspace(0,1,2), blue_purple, linspace(0,1,128));
yellow_red = [1 1 0; 1 0 0];
yellow_red = interp1(linspace(0,1,2), yellow_red, linspace(0,1,128));
combined_cmap = [blue_purple; yellow_red];

% Plot
h1 = scatter(odom_x, odom_y, 10, odom_z_norm, 'fill');
h2 = scatter(pose_x, pose_y, 10, pose_z_norm, 'fill');
colormap(gca, combined_cmap);  % Apply to axes
clim([1 256]);  % Full range
colorbar;

% % Extract exact start/end colors (index 1 & 128 for each half)
odom_start_col = combined_cmap(1, :);      % Blue
odom_end_col   = combined_cmap(64, :);     % Mid-purple transition, or use 128 for end
pose_start_col = combined_cmap(129, :);    % Yellow
pose_end_col   = combined_cmap(256, :);    % Red



% % Invisible legend scatters (same size/style as main)
leg1 = scatter(NaN, NaN, 5, 'o', 'MarkerFaceColor', odom_start_col, 'MarkerEdgeColor', 'none', 'Visible', 'on','DisplayName','a');
leg2 = scatter(NaN, NaN, 5, 'o', 'MarkerFaceColor', odom_end_col, 'MarkerEdgeColor', 'none', 'Visible', 'on','DisplayName','b');
leg3 = scatter(NaN, NaN, 5, 'o', 'MarkerFaceColor', pose_start_col, 'MarkerEdgeColor', 'none', 'Visible', 'on','DisplayName','c');
leg4 = scatter(NaN, NaN, 5, 'o', 'MarkerFaceColor', pose_end_col, 'MarkerEdgeColor', 'none', 'Visible', 'on', 'DisplayName','d');
% 
% Custom legend (exclude main h1/h2)
leg = legend([leg1, leg2, leg3, leg4], ...
             {'Odom Start', 'Odom End', 'Pose Start', 'Pose End'}, ...
             'Location', 'best');


hold off;
grid on;
axis equal;
xlabel('X Position (m)');
ylabel('Y Position (m)');
title(sprintf('Robot %i Trajectory (Odom vs Pose)', i));
xlim(xlim_centered);
ylim(ylim_centered);
set(gca, 'FontSize', 12);

%legend({'Odom','Pose'}, 'Location','best');


end



% function hFig = odomFigure(odomT, poseT, range, i, plotVisible)
% % ODOMFIGURE Plots the trajectory from a timetable with pos_x and pos_y columns.
% %   odomFigure(odomT, range_x, range_y) plots the trajectory and centers the axis
% %   limits so the trajectory is in the middle of a box with width range_x and height range_y.
% 
%     % Check input
%     if ~istimetable(odomT)
%         error('Input must be a timetable.');
%     end
%     if ~all(ismember({'pos_x','pos_y'}, odomT.Properties.VariableNames))
%         error('Timetable must contain variables ''pos_x'' and ''pos_y''.');
%     end
%     if ~exist("plotVisible","var")
%         plotVisible = 'on'; % default to show
%     end
% 
%     % Extract data
%     odom_x = odomT{:,'pos_x'};
%     odom_y = odomT{:,'pos_y'};
%     pose_x = poseT{:,'pos_x'};
%     pose_y = poseT{:,'pos_y'};
% 
%     % Compute center
%     mid_x = (min([odom_x;pose_x]) + max([odom_x;pose_x])) / 2;
%     mid_y = (min([odom_y;pose_y]) + max([odom_y;pose_y])) / 2;
% 
%     % Compute axis limits
%     xlim_centered = [mid_x - range/2, mid_x + range/2];
%     ylim_centered = [mid_y - range/2, mid_y + range/2];
% 
%     % Plot
%     hFig = figure('visible',plotVisible,"Theme","Light");
%     plot(odom_x, odom_y, 'b-', 'LineWidth', 1.5);
%     hold on;
%     scatter(odom_x(1), odom_y(1), 60, 'g', 'filled', 'DisplayName', 'Start');
%     scatter(odom_x(end), odom_y(end), 60, 'r', 'filled', 'DisplayName', 'End');
%     hold off;
%     grid on;
%     axis equal;
%     xlabel('X Position (m)');
%     ylabel('Y Position (m)');
%     title(sprintf('Robot %i Odometry Trajectory',i));
%     xlim(xlim_centered);
%     ylim(ylim_centered);
%     legend('Trajectory', 'Start', 'End', 'Location', 'best');
%     set(gca, 'FontSize', 12);
% end
% 

%[appendix]{"version":"1.0"}
%---
