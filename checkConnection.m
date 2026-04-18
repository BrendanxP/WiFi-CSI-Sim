function connected = checkConnection(IP)
%%% Ping both IPs to check network connection
    % Checks for Windows or Linux
    if ispc
        ping = evalc("!ping -n 1 " + IP);
    elseif isunix
        ping = evalc("!ping -c 1 " + IP);
    end
    loss = regexp(ping, '([0-9]*)%.*loss', 'tokens');
    connected = ~isempty(loss) && str2double(loss{1}{1})==0;
end

%[appendix]{"version":"1.0"}
%---
