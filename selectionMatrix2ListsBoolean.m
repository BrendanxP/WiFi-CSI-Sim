function out = selectionMatrix2ListsBoolean(boolean_matrix, numR)
    % input is bottom left diagonal matrix
    % output is this converted for each individual robot

    boolean_matrix = boolean_matrix(1:numR-1,1:numR-1);
    out = false(numR,numR);
    out(2:end,1:end-1) = boolean_matrix;
    out(1:end-1,2:end) = logical(boolean_matrix' + out(1:end-1,2:end));
    out = mat2cell(out,[ones(1,numR)]);
end

%[appendix]{"version":"1.0"}
%---
