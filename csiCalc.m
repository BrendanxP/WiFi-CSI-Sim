function csi_noisy = csiCalc(txPosition,rxPosition,deltaTime,centerFrequency,bandwidth,numSubcarriers,deltaFrequency,SNR)
% Example input:
%txPosition = [0, 0, 0]; % Transmitter position
%rxPosition = [1, 0, 0]; % Receiver position

%%% Example to calculate exact channel information.
% See function channelFrequency for all details.
%channel = 108;
%[centerFrequency,bandwidth] = channelFrequency(channel);
%numSubcarriers = 30;

%%% Example to manually define channel information.
%Center WiFi frequency channel 108 at 5GHz as configured in config.json
%centerFrequency = 5540e6;   % central fequency 5.540GHz (5530-5550)
%bandwidth = 20e6;           % bandwidth 20 MHz
%numSubcarriers = 30;        % subcarriers 30

    % Calculate frequencies
    subcarrierFrequencies = linspace(centerFrequency - bandwidth/2, ...
        centerFrequency + bandwidth/2, numSubcarriers);

    % Calculate distance between transmitter and receiver at time t
    distance = norm(txPosition - rxPosition);

    % Calculate wavelength
    lambda = physconst('LightSpeed') ./ subcarrierFrequencies; % Speed of light =~ 3e8 m/s

    % Calculate CSI phase and amplitude
    csi_amplitude = 1 / distance;
    csi_phase = -2 * pi * distance ./ lambda;

    % Calculate the CFO for both nodes
    CFO = 2 * pi * deltaFrequency * deltaTime;
    % OPTIONAL: add small (variable) delay to deltaTime for TX
    %deltaTime = deltaTime + ( -0.0001 + -0.0002*rand(1,1)); 
    %CFO_TX = -2 * pi * deltaFrequency * deltaTime;

    csi_phase_noisy = awgn(csi_phase,SNR);

    % Add CFO and calculate signal
    csi_noisy = csi_amplitude .* exp(1i .* (csi_phase_noisy + CFO));

    % % Add noise to signal based on noise to signal ratio
    % csi_noisy = awgn(csi_clean,SNR,'measured');
end