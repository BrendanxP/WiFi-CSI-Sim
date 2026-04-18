function [p_error_list, p_est_list, p_true_list, R, hFig1, hFig2] = localizeAndPlotOld(R, allPos, receivers, transmitters, AoA_est, plotVisible, addStartPos2Odom)
% LOCALIZEANDPLOT Perform AoA-based localization and visualization
% Inputs:
%   R            - Struct array with fields:
%                  .odomT (timetable: pos_x, pos_y)
%                  .comDevices (logical array robot communications)
%   allPos       - [Nx2] matrix of ground truth positions
%   receivers    - [1xN] logical array (true for mobile robots)
%   transmitters - [1xN] logical array (true for static robots)
%   AoA_est      - [numReceivers x numTransmitters] matrix of AoA estimates (degrees)
%
% Outputs:
%   p_est    - [numReceivers x 2] estimated positions
%   p_error  - [numReceivers x 1] localization errors

    if ~exist("plotVisible","var")
        plotVisible = 'on'; % default to show
    end
    
    checkAllCombinations = true;
    
    AoA_est_all=AoA_est;
    
    % Extract positions
    p_true_list = allPos(receivers, 1:2);
    p_transmitters = allPos(transmitters, 1:2);
    numR = nnz(receivers);  % Number of receivers
    numT = nnz(transmitters);  % Number of transmitters
    
    % Preallocate results
    %p_est = cells(numR, 3);
    
    % Localization via optimization
    for i = find(receivers)
        if checkAllCombinations
            % All combinations of 2+ transmitters
            combinationsList = generate_transmitter_combinations(R(i).comDevices); % cell array of logical lists indicating participating robots
            p_results = table('Size',[length(combinationsList),5], ...
                'VariableTypes',["double","string","double","double","double"], ...
                'VariableNames',["combID","combList","p_est_x","p_est_y","p_error"]);
            for j=1:length(combinationsList)
    
                theta = deg2rad(AoA_est_all(i, combinationsList{j}));
                n_vectors = [cos(theta); sin(theta)]';
    
                combList = num2str(combinationsList{j}); % string representation of the logical list
                combID = bin2dec(combList); % binary representation of the boolean array
                options = optimoptions('lsqnonlin', 'Display', 'off', 'Algorithm', 'levenberg-marquardt');
                p_est = lsqnonlin(@(p) dist_to_lines(p, allPos(combinationsList{j},1:2), n_vectors), ...
                                        mean(allPos(combinationsList{j},1:2)), [], [], options);
                p_error = vecnorm(p_est - allPos(i,1:2), 2, 2); 
                p_results(j,:) = {combID,combList,p_est(1),p_est(2),p_error};
            end
            p_est_list(sum(receivers(1:i)),:) = p_est; % store last entry with all connections as main source
            R(i).p_results=p_results;
        else
            theta = deg2rad(AoA_est_all(i, :));
            activeCom = ~isnan(theta);
            n_vectors = [cos(theta(activeCom)); sin(theta(activeCom))]';
            
            options = optimoptions('lsqnonlin', 'Display', 'off', 'Algorithm', 'levenberg-marquardt');
            p_est_list(i,:) = lsqnonlin(@(p) dist_to_lines(p, p_transmitters(activeCom,:), n_vectors), ...
                                    mean(p_transmitters(activeCom,:)), [], [], options);
        end
    end
    
    % Calculate errors
    p_error_list = vecnorm(p_est_list - p_true_list, 2, 2);
    

%% Plot 1 setup overview
    % Visualization
    hFig1 = figure("Visible",plotVisible,"Position",[909 320 560 350],"Theme","Light"); 
    hold on; grid on; axis equal;
    %title('Multi-Robot Localization using AoA');
    xlabel('X Position (m)'); 
    ylabel('Y Position (m)');
    
    % 3. Plot ground truth connections
    commAll = reshape([R.comDevices], numel(R), []);
    commRT = commAll(receivers, transmitters);
    [recIdx, transIdx] = find(commRT);
    x_lines = [p_transmitters(transIdx,1), p_true_list(recIdx,1)]';
    y_lines = [p_transmitters(transIdx,2), p_true_list(recIdx,2)]';
    plot(x_lines, y_lines, 'r:', 'LineWidth', 1, 'HandleVisibility', 'off');
    if numel(R)<=4 % hardcoded test with 4 of 6 experiments
        % Handle legend nicely (only when no other lines need to be drawn).
        plot(nan, nan, 'r:', 'LineWidth', 1, 'DisplayName', 'Ground Truth');
    else
        % manually draw line between two moding nodes(4 and 5).
        plot(allPos(4:5,1), allPos(4:5,2), 'r:', 'LineWidth', 1, 'DisplayName', 'Ground Truth');
    end

    % 1. Plot mobile robot trajectories
    for i = find(receivers)
        odom = R(i).odomT{:, ["pos_x","pos_y"]};
        if addStartPos2Odom
            odom = odom + allPos(i,1:2); % add start pos to odom if needed
        end
        plot(odom(:,1), odom(:,2), 'LineWidth', 1.5, ...
            "DisplayName", "Odom R" + i);
    end
    
    % 4. Plot static robots
    scatter(p_transmitters(:,1), p_transmitters(:,2), 30, 'filled', ...
            'DisplayName', 'Static Robots');
    transmitterNames = "Static " + string(find(transmitters));
    text(p_transmitters(:,1), p_transmitters(:,2)+0.2, transmitterNames, ...
         'VerticalAlignment','bottom', 'HorizontalAlignment','center', ...
         'FontSize',10,'FontWeight','bold');
    
    % 5. Plot estimated/true positions
    % scatter(p_est_list(:,1), p_est_list(:,2), 120, 'y*', 'LineWidth', 1, ...
    %         'DisplayName', 'Est Pos');
    % scatter(p_true_list(:,1), p_true_list(:,2), 150, 'pentagram', 'LineWidth', 1, ...
    %         'DisplayName', 'True Pos');
    receiverNames = "Mobile " + string(find(receivers));
    text(p_true_list(:,1), p_true_list(:,2)+0.2, receiverNames, ...
         'VerticalAlignment','bottom', 'HorizontalAlignment','center', ...
         'FontSize',10,'FontWeight','bold');
    
    % Set axis limits to best fit the nodes
    xlim_min = min(allPos(:,1));
    xlim_max = max(allPos(:,1));
    xlim_range = xlim_max - xlim_min;
    ylim_min = min(allPos(:,2));
    ylim_max = max(allPos(:,2));
    ylim_range = ylim_max - ylim_min;
    
    xlim_min = xlim_min - xlim_range * 0.1;
    xlim_max = xlim_max + xlim_range * 0.1;
    ylim_min = ylim_min - ylim_range * 0.2;
    ylim_max = ylim_max + ylim_range * 0.2;
    
    xlim([xlim_min, xlim_max])
    ylim([ylim_min, ylim_max])
    
    % Finalize plot
    legend('Location', 'southoutside','NumColumns',2);
    set(gca, 'FontSize', 12);

%% Plot 2 localisation performance
    % Visualization
    hFig2 = figure("Visible",plotVisible,"Position",[909 320 560 350],"Theme","Light"); 
    hold on; grid on; axis equal;
    %title('Multi-Robot Localization using AoA');
    xlabel('X Position (m)'); 
    ylabel('Y Position (m)');
    
    % 2. Plot AoA rays
    ray_length = 7;  % Meters
    for i = find(receivers)
        theta = deg2rad(AoA_est_all(i,R(i).comDevices)); % With reversed bearing (+180deg) (static->moving)
        dir_vec = [cos(theta(:)), sin(theta(:))];  % Direction calculation
        
        p_communicators = allPos(R(i).comDevices,1:2);
    
        % Draw rays
        quiver(p_communicators(:,1), p_communicators(:,2), ...
            dir_vec(:,1)*ray_length, dir_vec(:,2)*ray_length, 0, ...
            'LineWidth', 1, 'MaxHeadSize', 0.1, 'DisplayName', "AoA R"+i);
    end
    
    % 3. Plot ground truth connections
    commAll = reshape([R.comDevices], numel(R), []);
    commRT = commAll(receivers, transmitters);
    [recIdx, transIdx] = find(commRT);
    x_lines = [p_transmitters(transIdx,1), p_true_list(recIdx,1)]';
    y_lines = [p_transmitters(transIdx,2), p_true_list(recIdx,2)]';
    plot(x_lines, y_lines, 'r:', 'LineWidth', 1, 'HandleVisibility', 'off');
    plot(nan, nan, 'r:', 'LineWidth', 1, 'DisplayName', 'Ground Truth');
    
    % 4. Plot static robots
    scatter(p_transmitters(:,1), p_transmitters(:,2), 30, 'filled', ...
            'DisplayName', 'Static Robots');
    transmitterNames = "Static " + string(find(transmitters));
    text(p_transmitters(:,1), p_transmitters(:,2)+0.2, transmitterNames, ...
         'VerticalAlignment','bottom', 'HorizontalAlignment','center', ...
         'FontSize',12,'FontWeight','bold');
    
    % 5. Plot estimated/true positions
    scatter(p_est_list(:,1), p_est_list(:,2), 30, '*', 'LineWidth', 1, ...
            'DisplayName', 'Est Pos');
    scatter(p_true_list(:,1), p_true_list(:,2), 30, 'pentagram', 'LineWidth', 1, ...
            'DisplayName', 'True Pos');
    receiverNames = "Mobile " + string(find(receivers));
    text(p_true_list(:,1), p_true_list(:,2)+0.2, receiverNames, ...
         'VerticalAlignment','bottom', 'HorizontalAlignment','center', ...
         'FontSize',12,'FontWeight','bold');
    
    % Set axis limits to best fit the nodes
    xlim_min = min(allPos(:,1));
    xlim_max = max(allPos(:,1));
    xlim_range = xlim_max - xlim_min;
    ylim_min = min(allPos(:,2));
    ylim_max = max(allPos(:,2));
    ylim_range = ylim_max - ylim_min;
    
    xlim_min = xlim_min - xlim_range * 0.1;
    xlim_max = xlim_max + xlim_range * 0.1;
    ylim_min = ylim_min - ylim_range * 0.2;
    ylim_max = ylim_max + ylim_range * 0.2;
    
    xlim([xlim_min, xlim_max])
    ylim([ylim_min, ylim_max])
    
    % Finalize plot
    legend('Location', 'southoutside','NumColumns',2);
    set(gca, 'FontSize', 12);
end


%[appendix]{"version":"1.0"}
%---
