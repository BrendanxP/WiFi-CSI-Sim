%[text] # Parse JSON NEXMON
%% Select JSON file
% Insert a Live Editor *file browser* control by:
% Live Editor tab → Control → File
% Then replace the variable name below with that control's variable, e.g. jsonFilePath

% Example default (will be replaced by the Live control)
jsonFilePath = "/home/brendan/Documents/CSI_FINAL/data/20260206_csi_run_21.json";   % <-- replace with file-control variable %[control:filebrowser:2e4e]{"position":[16,81]}

% Basic checks
assert(isstring(jsonFilePath) || ischar(jsonFilePath), ...
    "Path must be a string or char.");
[jsonFolder, jsonName, jsonExt] = fileparts(jsonFilePath);
assert(strcmpi(jsonExt,".json"), "Selected file must have .json extension.");

% Read and decode JSON
rawText   = fileread(jsonFilePath);          % [web:12]
jsonData  = jsondecode(rawText);             % [web:6][web:9]

%% Convert to struct / table
% Assume JSON has top-level fields: meta, channel_packets, etc.
meta = [];
channelPackets = [];

if isfield(jsonData,"meta")
    meta = jsonData.meta;                    % struct with metadata [web:6]
end

if isfield(jsonData,"channel_packets")
    channelPackets = jsonData.channel_packets;

    if isstruct(channelPackets) && isscalar(channelPackets)
        % struct of structs → struct array
        cellPackets = struct2cell(channelPackets);  % [web:51]
        packetsStruct = [cellPackets{:}];           % 1×N struct array [web:45]
    elseif isstruct(channelPackets) && ~isscalar(channelPackets)
        % already a struct array
        packetsStruct = channelPackets;
    elseif iscell(channelPackets)
        % cell array of structs
        packetsStruct = [channelPackets{:}];
    else
        packetsStruct = struct([]);
    end
else
    packetsStruct = struct([]);
end

% Optional table
if ~isempty(packetsStruct)
    packetTable = struct2table(packetsStruct);      % [web:44]
else
    packetTable = table();
end


%% Prepare dropdown variables for Live Script controls
% For Live Editor drop-downs:
% Live Editor → Control → Drop Down, bind to variables channelList and packetIndexList

% Channel list (e.g., channel_packets entry index)
nPackets = numel(packetsStruct);
channelList = "Channel " + string(0:nPackets-1);   % 0..X labeling

% Packet index list (same indices, but numeric)
packetIndexList = 0:nPackets-1;

% Insert two drop-down controls later that write into:
%   selectedChannel   (string from channelList)
%   selectedPacketIdx (double from packetIndexList)

selectedChannel   = channelList(1);    % default
selectedPacketIdx = packetIndexList(1);

%% Access selected packet fields
% Map selectedPacketIdx (0..X) to struct index (1..X+1)
idx = selectedPacketIdx + 1;
assert(idx >= 1 && idx <= nPackets, "Selected packet index out of range.");

pkt = packetsStruct(idx);

% Expected fields in each packet:
% self_id, peer_id, rssi_self, rssi_peer, timestamp, t_ns,
% csi_self_iq_b64, csi_peer_iq_b64

self_id         = getfield(pkt, "self_id");          %#ok<GFLD>
peer_id         = getfield(pkt, "peer_id");
rssi_self       = getfield(pkt, "rssi_self");
rssi_peer       = getfield(pkt, "rssi_peer");
timestamp       = getfield(pkt, "timestamp");
t_ns            = getfield(pkt, "t_ns");
csi_self_iq_b64 = getfield(pkt, "csi_self_iq_b64");
csi_peer_iq_b64 = getfield(pkt, "csi_peer_iq_b64");

%% Example: decode base64 CSI strings (placeholder)
% Here you can plug in your own base64/CSI decoding.
% For now, just keep them as strings:
csiSelfRaw = csi_self_iq_b64;
csiPeerRaw = csi_peer_iq_b64;

% Decode base64 CSI strings into numeric arrays
    csiSelfDecoded = matlab.net.base64decode(csiSelfRaw);
    csiPeerDecoded = matlab.net.base64decode(csiPeerRaw);


%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright"}
%---
%[control:filebrowser:2e4e]
%   data: {"browserType":"File","defaultValue":"\"\"","label":"JSON","run":"Nothing"}
%---
