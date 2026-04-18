function ppData = preProcessBartlett(CSI_i, CSI_j, odom_i, MAC_i, MAC_j, movingWindow, numSubC)

    if ~exist("plotVisible","var")
        plotVisible = 'on'; % default to show
    end

    %% Process CSI data to filter on node, moving window and matching pairs
    CSI_i = filterMAC(CSI_i, MAC_j);           % Filter CSI data on other node's MAC
    CSI_i = filterWindow(CSI_i, movingWindow); % CSI data within moving window
    CSI_i = CSI_i(:,["itr","CSI"]);            % Remove redundant data
    CSI_i = renamevars(CSI_i,"CSI","CSI_i");   % Rename for merging
    
    CSI_j = filterMAC(CSI_j, MAC_i);           % Filter CSI data on other node's MAC
    CSI_j = filterWindow(CSI_j, movingWindow); % CSI data within moving window
    CSI_j = CSI_j(:,["itr","CSI"]);            % Remove redundant data
    CSI_j = renamevars(CSI_j,"CSI","CSI_j");   % Rename for merging

    if isempty(CSI_i) || isempty(CSI_j)
        ppData = [];
        warning("missing data! did the nodes communicate?")
        return; % Exit the function if either CSI_i or CSI_j is empty
    end
    
    % In new versions of MATLAB this is no longer required.
    %CSI_j = timetable2table(CSI_j); % Only left join table can remain as timetable and keep datetime info
    
    % Inner join to only keep matching pairs and keep datetime of node i
    ppData = innerjoin(CSI_i,CSI_j,"Keys",["itr","itr"]);

    % Check if all times are unique
    orig_n = numel(ppData.Time);
    [uniq_l, unique_idx] = unique(ppData.Time);
    uniq_n = numel(uniq_l);
    
    if orig_n ~= uniq_n
        ppData = ppData(unique_idx,:);
        warning("removed %i entries because identical timestamp",orig_n-uniq_n)
    end
    
    %% Process odom data to match CSI
    odom_i = retime(odom_i,ppData.Time,'linear');      % interpolated odom to CSI timestamps
    odom_i = renamevars(odom_i,["pos_x","pos_y","pos_z","quat_x","quat_y","quat_z","quat_w"], ...
        ["pos_xi","pos_yi","pos_zi","quat_xi","quat_yi","quat_zi","quat_wi"]); % rename for merging
    ppData = [ppData,odom_i];                                                % merge with CSI data
    
    % Convert to Euler angles (ZYX order)
    eul = quat2eul([ppData.quat_wi, ppData.quat_xi, ...
        ppData.quat_yi, ppData.quat_zi], "ZYX");
    
    % Assign to CSI_ij
    ppData.eul_zi = eul(:,1);
    ppData.eul_yi = eul(:,2);
    ppData.eul_xi = eul(:,3);
    
    % Centre subcarriers for magniture
    magSubC = floor(numSubC/2):ceil(numSubC/2);
                   
    % Define interpolation function for a single CSI array
    interpolateCSI = @(csi) (...
        mean(abs(csi(magSubC))) * ...                     % Average magnitude
        exp(1i * polyval( ...                             % Phase
            polyfit(1:numSubC, unwrap(angle(csi)), 1), (numSubC+1)/2) ...
        ) ...
    );

    % Process timetable in one step
    ppData(:,["CSI_int_i","CSI_int_j"]) = rowfun(...
        @(csi_i, csi_j) deal(...
            interpolateCSI(csi_i), ...  % CSI_int_i
            interpolateCSI(csi_j) ...   % CSI_int_j
        ), ...
        ppData, ...
        'InputVariables', {'CSI_i', 'CSI_j'}, ...
        'OutputVariableNames', {'CSI_int_i', 'CSI_int_j'}, ...
        'ExtractCellContents', true ... % Required if CSI_i/j are cell arrays
    );
    
    % Cancel CFO
    ppData(:,"CSI") = rowfun(@(i,j) i*j, ppData, ...  % Multiply CSI_int_i and CSI_int_j
        'InputVariables', {'CSI_int_i', 'CSI_int_j'}, ...
        'OutputVariableName', 'CSI');
    
    % Positional input Bartlett
    ppData.yawList = atan2( ...
        ppData.pos_yi - ppData.pos_yi(1), ...   % azimuth angles of rx and rx's initial position
        ppData.pos_xi - ppData.pos_xi(1));
    
    ppData.pitchList = ppData.eul_yi - ppData.eul_yi(1);  % elevation angles of rx and rx's initial position
    
    ppData.rhoList = sqrt( ...
        (ppData.pos_xi - ppData.pos_xi(1)).^2 + ...
        (ppData.pos_yi - ppData.pos_yi(1)).^2 + ...
        (ppData.pos_zi - ppData.pos_zi(1)).^2);           % distance between rx and rx's initial position

end

function outTable = filterMAC(inTable, MAC)
    % Filter the CSI data based on MAC addresses
    outTable = inTable(strcmp(inTable.MAC, MAC), :);
end

function outTable = filterWindow(inTable, window)
    % Define the output table structure for filtered data
    outTable = inTable(max(1, height(inTable) - window):end, :);
end


%[appendix]{"version":"1.0"}
%---
