function [centerFrequency,bandwidth] = channelFrequency(channel)
% This functions gives the centerFrequency and bandwith based on the WiFi
% channel number.
% Specify the channel using the following format:
% 2.4GHz 20MHz    1: 1:13
% 2.4GHZ 40MHz  1.5: 1:12.5
%   5GHz 20MHz   32: 4:144
%   5GHz 40MHz   38: 8:142
%   5GHz 80MHz   42:16:138
%   5GHz 160MHz  50:32:114

% https://en.wikipedia.org/wiki/List_of_WLAN_channels
    
    % Check inputs
    if ~any(channel==[1:0.5:13,32,36:2:144])
        error("channel out of range 2.4GHz 1:1:13 or 5GHz 32:2:144 (excl. 34).")
    end   

    % 2.4GHz - 20MHz (1:1:13) - Up to 64 subcarriers
    if channel<=13 && round(channel)==channel
        bandwidth = 20e6;
        centerFrequency = 2412e6 + 5e6 * (channel-1);
    % 2.4GHz - 40MHz (1.5:1:12.5) - Up to 128 subcarriers
    elseif channel<=12.5
        bandwidth = 40e6;
        centerFrequency = 2414e6 + 5e6 * (channel-1.5); %prevent 0.5e6
    % 5GHz - 160MHz (50:32:114) - Up to 512 subcarriers
    elseif mod(channel-18,32)==0
        bandwidth = 160e6;
        centerFrequency = 5250e6 + (160e6 * (channel-50)/32);
    % 5GHz - 80MHz (42:16:138) - Up to 256 subcarriers
    elseif mod(channel-10,16)==0
        bandwidth = 80e6;
        centerFrequency = 5210e6 + ( 80e6 * (channel-42)/16);
    % 5GHz - 40MHz (38:8:142) - Up to 128 subcarriers
    elseif mod(channel-6,8)==0
        bandwidth = 40e6;
        centerFrequency = 5190e6 + ( 40e6 * (channel-38)/8);
    % 5GHz - 20MHz (32:4:144) - Up to 64 subcarriers
    elseif mod(channel,4)==0
        bandwidth = 20e6;
        centerFrequency = 5160e6 + ( 20e6 * (channel-32)/4);
    end
end