function out = selectionMatrix2ListsScaling(boolean_matrix, numR, true_value, false_value, comLists)
    % Input boolean left diagonal matrix.
    % Output is lists of SNR set to low or high for communicating devices,
    % and 0 for devices that do no communicate.

    boolean_matrix = boolean_matrix(1:numR-1,1:numR-1);
    out = false_value * (ones(numR,numR) - diag(ones(1,numR))); % set all options to low
    out(2:end,1:end-1) = boolean_matrix * (true_value-false_value) + out(2:end,1:end-1); % update bottom left to high where true
    out(1:end-1,2:end) = boolean_matrix' * (true_value-false_value) + out(1:end-1,2:end); % update top right to high where true
    out = out .* cell2mat(comLists); % only keep SNR at active connections
    out = mat2cell(out,[ones(1,numR)]); % make lists for each robot
end

%[appendix]{"version":"1.0"}
%---
