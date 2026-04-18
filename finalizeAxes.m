function finalizeAxes(allPos)
% Set consistent axis limits, legend, and font for current axes

% Compute limits with padding
xlim_min   = min(allPos(:,1));
xlim_max   = max(allPos(:,1));
xlim_range = max(xlim_max - xlim_min, 0.001); % make sure its never 0
ylim_min   = min(allPos(:,2));
ylim_max   = max(allPos(:,2));
ylim_range = max(ylim_max - ylim_min, 0.001); % make sure its never 0

xlim([xlim_min - 0.1*xlim_range, xlim_max + 0.1*xlim_range]);
ylim([ylim_min - 0.2*ylim_range, ylim_max + 0.2*ylim_range]);

% Legend and font (standard for both plots)
legend('Location', 'southoutside', 'NumColumns', 2);
set(gca, 'FontSize', 12);
end


%[appendix]{"version":"1.0"}
%---
