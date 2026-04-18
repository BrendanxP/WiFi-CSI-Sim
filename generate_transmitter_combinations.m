function boolCombinations = generate_transmitter_combinations(transmitters)
    activeIdx = find(transmitters);
    boolCombinations = {};
    if isempty(activeIdx)
        warning("could not find any transmitter combinations");
        return;
    end
    nActive = numel(activeIdx);
    
    
    % Pre-calculate total combinations
    totalCombs = 0;
    for k = 1:nActive
        totalCombs = totalCombs + nchoosek(nActive, k);
    end
    
    % Preallocate cell array
    boolCombinations = cell(1, totalCombs);
    counter = 1;
    
    for k = 1:nActive
        comb = nchoosek(activeIdx, k);
        for i = 1:size(comb, 1)
            newArray = false(size(transmitters));
            newArray(comb(i,:)) = true;
            boolCombinations{counter} = newArray;
            counter = counter + 1;
        end
    end
end

%[appendix]{"version":"1.0"}
%---
