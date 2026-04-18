function [p_error_list, p_est_list, p_true_list, p_transmitters, p_results_list] = localizeAndPlot( ...
        R, allPos, receivers, transmitters, AoA_est)

checkAllCombinations = true;
AoA_est_all = AoA_est;

% Extract positions
p_true_list      = allPos(receivers, 1:2);
p_transmitters   = allPos(transmitters, 1:2);
p_results_list   = cell(numel(R),1);

% Localization via optimization
for i = find(receivers)
    if checkAllCombinations
        % All combinations of 2+ transmitters
        combinationsList = generate_transmitter_combinations(R(i).comDevices);
        if isempty(combinationsList)
            warning("receiver did not communicate with other nodes")
            break;
        end
        p_results = table('Size',[length(combinationsList),5], ...
            'VariableTypes',["double","string","double","double","double"], ...
            'VariableNames',["combID","combList","p_est_x","p_est_y","p_error"]);
        for j = 1:length(combinationsList)
            theta    = deg2rad(AoA_est_all(i, combinationsList{j}));
            n_vectors = [cos(theta); sin(theta)]';
            combList = num2str(combinationsList{j}); % string representation
            combID   = bin2dec(combList);            % binary representation
            options = optimoptions('lsqnonlin', 'Display', 'off', ...
                                   'Algorithm', 'levenberg-marquardt');
            p_est = lsqnonlin(@(p) dist_to_lines(p, ...
                           allPos(combinationsList{j},1:2), n_vectors), ...
                           mean(allPos(combinationsList{j},1:2),1), [], [], options);
            p_error = vecnorm(p_est - allPos(i,1:2), 2, 2);
            p_results(j,:) = {combID,combList,p_est(1),p_est(2),p_error};
        end
        p_est_list(sum(receivers(1:i)),:) = p_est; % last entry is with all connections
        p_results_list{i} = p_results;
    else
        theta     = deg2rad(AoA_est_all(i, :));
        activeCom = ~isnan(theta);
        n_vectors = [cos(theta(activeCom)); sin(theta(activeCom))]';
        options = optimoptions('lsqnonlin', 'Display', 'off', ...
                               'Algorithm', 'levenberg-marquardt');
        p_est_list(i,:) = lsqnonlin(@(p) dist_to_lines(p, ...
                           p_transmitters(activeCom,:), n_vectors), ...
                           mean(p_transmitters(activeCom,:)), [], [], options);
    end
end

% Calculate errors
p_error_list = vecnorm(p_est_list - p_true_list, 2, 2);



end


%[appendix]{"version":"1.0"}
%---
