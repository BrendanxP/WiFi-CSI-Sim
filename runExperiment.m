function R = runExperiment(R,rate,centerFrequency,bandwidth,numSubC,RosVersion,truePoseSub,loadingBar)

% Toggle debug printing
debug = false;

numR = numel(R);

% List of all nodes that are actively communicating
commNodes = nonzeros(any(reshape([R.comDevices],numR,[])).*(1:numR))';

% predefine list
truePose = nan(5,7);

% take not of real start time
startTime = datetime('now');
    
% end time
tEnd = numel(R(1).vx);

% Create waitbar
if loadingBar
    h = waitbar(0, 'Starting simulation...', 'Name', 'Progress', 'WindowStyle','alwaysontop','units','pixels', 'Position', [100,100, 800, 100]);
end

pause(1)

% loop over time
for t = 1:tEnd

    %%% Subscribe Odom
    % Save position and orientation for all actively communicating nodes
    for i = commNodes
        %% Subscribe
        try
            msgSub = R(i).rosSub.receive(1);
        catch
            msgSub = R(i).rosSub.receive(1);
        end
        
        % Get odom from the message
        [pos, ori, simTime, isOK] = extractROSMessage(msgSub,RosVersion);

        % Check ROS odom data
        if ~isOK
            warning("NaN value appeared on iteration:" + i);
            continue; % skip this iteration
        end
        
        % Store odom at node in timetable
        odomTime = startTime + seconds(simTime);
        R(i).odomT(odomTime,:) = [pos(2:end), ori(2:end)];
    end

    %%% Subscribe true pose
    try
        msgPose = receive(truePoseSub,1);
    catch
        msgPose = receive(truePoseSub,1);
    end
    
    % Time for all true poses are identical from the same packet
    poseTime = double(msgPose.header.stamp.sec)+double(msgPose.header.stamp.nanosec)*1e-9; % simulation time
    poseTime = startTime + seconds(poseTime);

    % Extract data from message and enter into pose timetable
    for i = commNodes
        pose = struct2cell(msgPose.poses(R(i).poseID).position)';
        orie = struct2cell(msgPose.poses(R(i).poseID).orientation)';
        % average odom and pose times to prevent having identical pose
        % times for all connections between the nodes.
        avgTime = mean([poseTime, R(i).odomT.Time(end)]);
        R(i).poseT(avgTime,:) = [pose(2:end),orie(2:end)];
    end
    
    %%% Calculate CSI for all Active Signal Communications
    % loop over all actively communicating nodes
    for i = commNodes
        % Loop over all its active connections
        commPartners = nonzeros([R(i).comDevices] .* (1:numR))';
        for j = commPartners
            % Printing debug
            if debug
                fprintf("Time %i, CSI %i to %i\n",t,i,j)
            end
            
            % Define input for CSI calc
            posRX = R(i).poseT{end, {'pos_x', 'pos_y', 'pos_z'}}; %+ R(i).posStart(1:3); % add start pos to odom, dont add to true pose.
            posTX = R(j).poseT{end, {'pos_x', 'pos_y', 'pos_z'}}; %+ R(j).posStart(1:3);
            %meanTime = mean([R(i).odomT.Time(end), R(j).odomT.Time(end)]); % average time between two odom measurement in seconds since the UNIX epoch
            meanTime = mean([R(i).poseT.Time(end), R(j).poseT.Time(end)]);
            %meanTime = R(i).poseT.Time(end);
            deltaFrequency = R(i).deltaFrequency - R(j).deltaFrequency; % combined antenna offset
            SNR = mean([R(i).SNR(j), R(j).SNR(i)]); % average SNR configured between the two nodes for this connection
            
            % Calculate CSI
            CSI = csiCalc(posRX, posTX, posixtime(meanTime),...
                centerFrequency,bandwidth,numSubC,deltaFrequency,SNR);
            
            % Store data at each robot
	        R(i).csiT(meanTime,:) = {  % store at different time than odom
                t, ...                        % itr counter
                R(j).MAC, ...                 % MAC address of other node
                CSI(:) ...                    % CSI values array and make sure its vertical array
            };
            %R(i).csiT
        end
    end
    

    %%% Live bearing calc for rendezvous
    if any(strcmp([R.trajType],"rendezvous"))
        
        % Rendezvous calculation interval settings
        rdvInterval = 100; % Update angle every X itrs
        rdvWindow = 500; % moving window of last X packets

        if mod(t,rdvInterval)==0

            % Freeze Gazebo and sim time for heavy calculations
            %pauseRequest = rosmessage(pauseClient);
            %call(pauseClient, pauseRequest);
            
            % Loop over all robots doing rendezvous
            for i = nonzeros(strcmp([R.trajType],"rendezvous").*(1:numel(R)))'

                % Rendezvous target robot
                j = R(i).trajRDVTarget;
                
                %%%% BARTLETT ESTIMATION FUNC HERE
    
            end
            %% update trajectory of the robot

        end
    end
            



    %%% Publish Trajectory
    % Update speeds for all actively communicating nodes
    for i = commNodes
        % Set the publish message
        R(i).msgPub = setROSTrajectory(R(i).msgPub, R(i).vx(t), R(i).vy(t), R(i).rz(t), RosVersion);

        % Send ROS update
        send(R(i).rosPub,R(i).msgPub);
    end

    % Update waitbar
    if loadingBar
        fraction = t / tEnd;
        waitbar(fraction, h);
    end

    % Make sure to not overload the system
    waitfor(rate);
end

%Stop robots
for i = commNodes
    % Set the publish message
    R(i).msgPub = setROSTrajectory(R(i).msgPub, 0, 0, 0, RosVersion);

    % Send ROS update
    send(R(i).rosPub,R(i).msgPub);
end

% Close waitbar
if loadingBar
    close(h)
end

end


%[appendix]{"version":"1.0"}
%---
