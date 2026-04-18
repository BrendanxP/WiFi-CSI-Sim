function out = randomBetween(a,b)

    % Two inputs or single array
    if nargin<2
        if all(size(a) == [1,2]) || all(size(a) == [2,1])
            b = a(2); a = a(1); % Assign a and b from the input array
        else
            error('Input must be two numbers or a 1x2/2x1 array.');
        end
    end

    % Get random value between a and b
    out = a + (b-a) * rand;
end