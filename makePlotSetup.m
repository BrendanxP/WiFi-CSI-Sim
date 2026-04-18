function hFig1 = makePlotSetup(R, bearingPos, receivers, transmitters, ...
                               p_true_list, p_transmitters, ...
                               plotVisible, addStartPos2Odom, spawnPos, odomT)

hFig1 = figure("Visible",plotVisible,"Position",[909 320 560 350], ...
               "Theme","Light");
hold on; grid on; axis equal;
xlabel('X Position (m)');
ylabel('Y Position (m)');

% Ground truth connections
commAll = reshape([R.comDevices], numel(R), []);
commRT  = commAll(receivers, transmitters);
[recIdx, transIdx] = find(commRT);
x_lines = [p_transmitters(transIdx,1), p_true_list(recIdx,1)]';
y_lines = [p_transmitters(transIdx,2), p_true_list(recIdx,2)]';
plot(x_lines, y_lines, 'r:', 'LineWidth', 1, 'HandleVisibility', 'off');

if numel(R) <= 4
    plot(nan, nan, 'r:', 'LineWidth', 1, 'DisplayName', 'Ground Truth');
else
    plot(bearingPos(4:5,1), bearingPos(4:5,2), 'r:', 'LineWidth', 1, ...
         'DisplayName', 'Ground Truth');
end

% Mobile robot trajectories
for i = find(receivers)
    odom = odomT{i}{:, ["pos_x","pos_y"]};
    if addStartPos2Odom
        odom = odom + spawnPos(i,1:2);
    end
    plot(odom(:,1), odom(:,2), 'LineWidth', 1.5, ...
         "DisplayName", "Odom R" + i);
end

% Static robots
scatter(p_transmitters(:,1), p_transmitters(:,2), 30, 'filled', ...
        'DisplayName', 'Static Robots');
transmitterNames = "Static " + string(find(transmitters));
text(p_transmitters(:,1), p_transmitters(:,2)+0.2, transmitterNames, ...
    'VerticalAlignment','bottom', 'HorizontalAlignment','center', ...
    'FontSize',10,'FontWeight','bold');

% True receiver positions
receiverNames = "Mobile " + string(find(receivers));
text(p_true_list(:,1), p_true_list(:,2)+0.2, receiverNames, ...
    'VerticalAlignment','bottom', 'HorizontalAlignment','center', ...
    'FontSize',10,'FontWeight','bold');

% Axis limits and legend
finalizeAxes(bearingPos);
end


%[appendix]{"version":"1.0"}
%---
