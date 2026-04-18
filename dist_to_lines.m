function F = dist_to_lines(p, a, n)
    F = zeros(size(a,1),1);
    for i = 1:size(a,1)
        diff = p - a(i,:);
        proj = dot(diff, n(i,:)) * n(i,:);
        F(i) = norm(diff - proj);
    end
end

%[appendix]{"version":"1.0"}
%---
