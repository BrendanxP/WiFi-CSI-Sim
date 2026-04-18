function createCopyLink(cmd, description)
% createCopyLink Generate clickable "Copy command" link
%   createCopyLink('your command here', 'Nice description')
    
    % Escape single quotes in command: ' → '''' 
    escaped_cmd = strrep(cmd, '''', '''''');

    % Escape DOUBLE quotes: " → "" (for bash/ROS args safety)
    escaped_cmd = strrep(escaped_cmd, '"', '''''');
    
    fprintf('<a href="matlab:clipboard(''copy'',''%s'');disp(''Copied: %s'');">%s</a>\n', ...
            escaped_cmd, description, description);
end


%[appendix]{"version":"1.0"}
%---
