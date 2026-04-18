function hFig2 = makePlotLocalize(R, allPos, receivers, transmitters, ...
                                  AoA_est_all, p_true_list, ...
                                  p_transmitters, p_est_list, plotVisible)


hFig2 = figure("Visible",plotVisible,"Position",[909 320 560 350], ...
               "Theme","Light");
hold on; grid on; axis equal;
xlabel('X Position (m)');
ylabel('Y Position (m)');

% AoA rays
ray_length = 7; % m
for i = find(receivers)
    theta = deg2rad(AoA_est_all(i, R(i).comDevices));
    dir_vec = [cos(theta(:)), sin(theta(:))];
    p_communicators = allPos(R(i).comDevices,1:2);

    quiver(p_communicators(:,1), p_communicators(:,2), ...
        dir_vec(:,1)*ray_length, dir_vec(:,2)*ray_length, 0, ...
        'LineWidth', 1, 'MaxHeadSize', 0.1, 'DisplayName', "AoA R"+i);
end

% Ground truth connections
commAll = reshape([R.comDevices], numel(R), []);
commRT  = commAll(receivers, transmitters);
[recIdx, transIdx] = find(commRT);
x_lines = [p_transmitters(transIdx,1), p_true_list(recIdx,1)]';
y_lines = [p_transmitters(transIdx,2), p_true_list(recIdx,2)]';
plot(x_lines, y_lines, 'r:', 'LineWidth', 1, 'HandleVisibility', 'off');
plot(nan, nan, 'r:', 'LineWidth', 1, 'DisplayName', 'Ground Truth');

% Static robots
scatter(p_transmitters(:,1), p_transmitters(:,2), 30, 'filled', ...
        'DisplayName', 'Static Robots');
transmitterNames = "Static " + string(find(transmitters));
text(p_transmitters(:,1), p_transmitters(:,2)+0.2, transmitterNames, ...
    'VerticalAlignment','bottom', 'HorizontalAlignment','center', ...
    'FontSize',12,'FontWeight','bold');

% Estimated / true positions
scatter(p_est_list(:,1), p_est_list(:,2), 30, '*', 'LineWidth', 1, ...
        'DisplayName', 'Est Pos');
scatter(p_true_list(:,1), p_true_list(:,2), 30, 'pentagram', ...
        'LineWidth', 1, 'DisplayName', 'True Pos');
receiverNames = "Mobile " + string(find(receivers));
text(p_true_list(:,1), p_true_list(:,2)+0.2, receiverNames, ...
    'VerticalAlignment','bottom', 'HorizontalAlignment','center', ...
    'FontSize',12,'FontWeight','bold');

% Axis limits and legend
finalizeAxes(allPos);
end


%[appendix]{"version":"1.0"}
%---
