function killSystemCmd(pids,force)

    arguments
        pids
        force(1,1) logical = 0;
    end

    if isempty(pids) || ~isnumeric(pids)
        return;
    end
    
    pids = pids(:);
    validPids = pids(pids > 0);
    if isempty(validPids)
        return;
    end
    
    if ~force
        % SIGTERM terminate
        for i = 1:length(validPids)
            pid = validPids(i);
            
            % Phase 1: SIGTERM the target + all its descendants
            system(sprintf('pkill -TERM -P %d 2>/dev/null', pid));  % Kill direct children
            system(sprintf('kill -TERM %d 2>/dev/null', pid));       % Kill target
            
            % Phase 2: Get full process tree and kill everything
            [~, treeOut] = system(sprintf('ps --no-headers -o pid,ppid -ax | grep %d | grep -v grep', pid));
            treePids = regexp(treeOut, '^(\\d+)', 'tokens', 'once');
            treePids = str2double(treePids);
            treePids = unique(treePids(treePids > 0));
            
            for tp = treePids'
                system(sprintf('kill -TERM %d 2>/dev/null', tp));
            end
        end
    else
        % Force kill
        for i = 1:length(validPids)
            pid = validPids(i);
            system(sprintf('pkill -KILL -P %d 2>/dev/null', pid));
            system(sprintf('kill -KILL %d 2>/dev/null', pid));
            
            [~, treeOut] = system(sprintf('ps --no-headers -o pid,ppid -ax | grep %d | grep -v grep', pid));
            treePids = regexp(treeOut, '^(\\d+)', 'tokens', 'once');
            treePids = str2double(treePids);
            treePids = unique(treePids(treePids > 0));
            
            for tp = treePids'
                system(sprintf('kill -KILL %d 2>/dev/null', tp));
            end
        end
    end

    pause(1);

    % Final check
    alive = checkPidsAlive(pids);
    if any(alive)
        ctemp=sprintf('%i ', pids(alive));
        warning("PIDs remained alive: %s",ctemp)
    end

    fprintf("All pids terminated\n")
end

