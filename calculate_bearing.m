function [azimuth, elevation] = calculate_bearing(tPos, rPos)
%CALCULATE_BEARING Compute azimuth and elevation from transmitter to receiver.
%   [AZIMUTH, ELEVATION] = CALCULATE_BEARING(TPOS, RPOS) returns the azimuth
%   and elevation angles (in degrees) from the point TPOS to the point RPOS.
%
%   TPOS and RPOS are position vectors in ENU coordinates:
%     TPOS = [x_t, y_t, z_t]
%     RPOS = [x_r, y_r, z_r]
%
%   If TPOS or RPOS contain only two elements [x, y], the z-coordinate is
%   assumed to be 0 (i.e., points lie in the horizontal plane z = 0).
%
%   Azimuth is measured in the horizontal plane, as the angle from the
%   positive x-axis (East) towards the positive y-axis (North), in the
%   range [0, 360) degrees.
%
%   Elevation is the angle above the horizontal plane, in the range
%   [0, 180] degrees.

    % --- Normalize input dimensions: allow [x,y] or [x,y,z] ---
    if numel(tPos) == 2
        tPos = [tPos(:).' 0];   % ensure row, append z = 0
    elseif numel(tPos) ~= 3
        error('tPos must contain 2 or 3 elements: [x y] or [x y z].');
    end

    if numel(rPos) == 2
        rPos = [rPos(:).' 0];   % ensure row, append z = 0
    elseif numel(rPos) ~= 3
        error('rPos must contain 2 or 3 elements: [x y] or [x y z].');
    end

    % --- Vector from transmitter to receiver in ENU coordinates ---
    delta_x = rPos(1) - tPos(1);   % East component
    delta_y = rPos(2) - tPos(2);   % North component
    delta_z = rPos(3) - tPos(3);   % Up component

    % --- Azimuth: angle in horizontal plane, from East towards North ---
    azimuth = atan2(delta_y, delta_x);      % radians
    azimuth = mod(rad2deg(azimuth), 360);   % convert to [0, 360) degrees
    azimuth = wrapTo180(azimuth);           % convert to [-180,180)

    % --- Elevation: angle above horizontal plane ---
    horizontal_range = hypot(delta_x, delta_y);        % sqrt(dx^2 + dy^2)
    elevation = atan2(delta_z, horizontal_range);      % radians
    elevation = rad2deg(elevation)+90;                 % degrees
end
