function [vx, vy, vz, rz] = makeTrajectory(traj, numR, num_points)

    % predefine variables
    vx = cell(numR,1); vy = vx; vz = vx; rz = vx;

    for i=1:numR
        switch traj.Type{i}
            case "line"
                % Line
                azimuthDir = 0; % azimuth direction in deg 0-360
                trajMagnitude = traj.VelocityLin{i};
                start_pos = [0;0;0]; %x,y
                end_pos = [cos(azimuthDir)*trajMagnitude; sin(azimuthDir)*trajMagnitude; 0]; %x,y
                waypoints = [start_pos, end_pos];
                [~, refVel, ~] = trapveltraj(waypoints, num_points,'PeakVelocity',0.1);
    
                % Store the velocities
                vx{i}=refVel(1,:); 
                vy{i}=refVel(2,:); 
                vz{i}=refVel(3,:);
                rz{i}=refVel(1,:)*0;

            case "circle"
                % Circle
                if traj.Omnidirectional{i}
                    % Omnidirectional
                    radius = traj.RadiusOmni{i};
                    theta = linspace(0, 2*pi, num_points);
                    %refPos = [radius*cos(theta); radius*sin(theta); zeros(1,num_points)];
                    
                    omega = 2*radius*2*pi/num_points;  % angular velocity per step
                    refVel = [-radius*sin(theta)*omega; radius*cos(theta)*omega; zeros(1,num_points)];
                    
                    % Store the velocities
                    vx{i}=refVel(1,:); 
                    vy{i}=refVel(2,:); 
                    vz{i}=refVel(3,:); 
                    rz{i}=refVel(1,:)*0;
                else
                    % Differential
                    %vx{i} = ones(1,num_points) * traj.VelocityLin{i};
                    pad = 10; % padding for slow start/end
                    temp = ones(1,num_points) * traj.VelocityLin{i};
                    temp(1:pad) = temp(1:pad)./(pad:-1:1);
                    temp(end-pad+1:end) = temp(end-pad+1:end)./(1:pad);
                    vx{i} = temp;
                    vy{i} = zeros(1,num_points); 
                    vz{i} = zeros(1,num_points);
                    temp = ones(1,num_points) * traj.VelocityRot{i};
                    temp(1:pad) = temp(1:pad)./(pad:-1:1);
                    temp(end-pad+1:end) = temp(end-pad+1:end)./(1:pad);
                    rz{i} = temp;
                end
            case "square"
                % Define as 4 times 1/8 drive, 1/8 turn.
                rot_num = floor(num_points/8);
                lin_num = ceil(num_points - 4*rot_num)/4; %could cause little overflow which will be removed
                
                % Trajectory start with lin
                lin_points = repmat([ones(1,lin_num), zeros(1,rot_num)],1,4);
                lin_points = lin_points(1:num_points); % remove extra
                rot_points = ~lin_points;

                % required rot velocity to turn 90deg in given time
                rot_vel = 90/rot_num/(2*pi);

                vx{i} = lin_points * traj.VelocityLin{i};
                vy{i} = zeros(1,num_points); 
                vz{i} = zeros(1,num_points);
                rz{i} = rot_points * rot_vel;
    
            case "static"
                blank = zeros(1,num_points);
    
                % Store the velocities
                vx{i}=blank; 
                vy{i}=blank; 
                vz{i}=blank; 
                rz{i}=blank;
                clearvars blank;

            case "random-walk"
                % 0-50% straight, 0-50% rotation, remaining % staight
                % make sure any part has at least 1.
                % assume not omnidirectional
                p1 = ceil((0.6 + rand()/2) * num_points/2); % between 30% and 55% of run
                p2 = ceil(randomBetween(30,100) + p1); % this gives roughly 45-135deg rot
                p3 = num_points;

                linSpeed = traj.VelocityLin{i} * (-1)^randi(2,1,1); % flip sign random
                rotSpeed = traj.VelocityRot{i} * (-1)^randi(2,1,1); % flip sign random

                % Part 1
                vx{i}(1:p1) = linSpeed;
                vy{i}(1:p1) = 0;
                vz{i}(1:p1) = 0;
                rz{i}(1:p1) = 0;

                % Part 2
                vx{i}(p1+1:p2) = 0;
                vy{i}(p1+1:p2) = 0;
                vz{i}(p1+1:p2) = 0;
                rz{i}(p1+1:p2) = rotSpeed;

                % Part 3
                vx{i}(p2+1:p3) = linSpeed;
                vy{i}(p2+1:p3) = 0;
                vz{i}(p2+1:p3) = 0;
                rz{i}(p2+1:p3) = 0;

            case "sin-wave"
                % similar to circle
                if traj.Omnidirectional{i}
                    error("WIP omnidirectional sin-wave")
                else
                    % Differential
                    %vx{i} = ones(1,num_points) * traj.VelocityLin{i};
                    pad = 10; % padding for slow start/end
                    temp = ones(1,num_points) * traj.VelocityLin{i};
                    temp(1:pad) = temp(1:pad)./(pad:-1:1); %slow start
                    temp(end-pad+1:end) = temp(end-pad+1:end)./(1:pad); %slow end
                    temp(end/2-pad+1:end/2+pad) = temp(end/2-pad+1:end/2+pad)./([1:pad,pad:-1:1]); %slow middle
                    vx{i} = temp;
                    vy{i} = zeros(1,num_points); 
                    vz{i} = zeros(1,num_points);
                    temp = ones(1,num_points) * traj.VelocityRot{i};
                    temp(1:pad) = temp(1:pad)./(pad:-1:1); %slow start
                    temp(end-pad+1:end) = temp(end-pad+1:end)./(1:pad); %slow end
                    temp(end/2-pad+1:end/2+pad) = temp(end/2-pad+1:end/2+pad)./([1:pad,pad:-1:1]); %slow middle
                    temp(end/2+pad:end) = -temp(end/2+pad:end); %flip second half
                    rz{i} = temp;
                end

            otherwise
                error("unknown traj type: %s", traj.Type{i})
        end
    end

end

%[appendix]{"version":"1.0"}
%---
