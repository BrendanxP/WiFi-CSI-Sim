function status = checkPidsAlive(pids)
    % Check if PIDs are running. Returns logical array (true=alive)
    % Usage: alive = checkPidsAlive(pids);
    
    if isempty(pids) || ~isnumeric(pids)
        status = [];
        return;
    end
    
    pids = unique(pids(pids > 0));  % Clean input
    status = false(size(pids));
    
    for i = 1:length(pids)
        pid = pids(i);
        
        % Method 1: Check /proc/PID exists (fastest, Linux only)
        if exist(sprintf('/proc/%d', pid), 'dir') == 7
            status(i) = true;
            continue;
        end
        
        % Method 2: ps -p PID (works everywhere)
        [~, psStatus] = system(sprintf('ps -p %d >/dev/null 2>&1', pid));
        status(i) = ~isempty(psStatus);
    end
end


