function figSignal = makePlotSignal(CSI, plotVisible)

    figSignal = figure("Visible",plotVisible,"Position",[909 320 560 200],"Theme","Light");
    % tiledlayout(2, 1); % Create a 2x1 tiled chart layout
    % 
    % % Amplitude / Magnitude plot in the first tile
    % ax1 = nexttile; % Get the axes for the first tile
    % csi_amplitude = abs(CSI_ij.CSI_cfo);
    % scatter(ax1, 1:length(csi_amplitude), csi_amplitude,".");
    % %xlabel(ax1, x_label);
    % ylabel(ax1, "Amplitude (-)");
    % %title(ax1, sprintf("Signal Amplitude Robot %i to %i",i,j));
    
    % Phase plot in the second tile
    ax2 = nexttile; % Get the axes for the second tile
    csi_phase = (angle(CSI));
    scatter(ax2, 1:length(csi_phase), csi_phase,".");
    %xlabel(ax2, x_label);
    ylabel(ax2, "Phase (rad)");
    ylim([-pi,pi])
    set(gca,'ytick',(-pi:pi/2:pi)) % where to set the tick marks
    set(gca,'yticklabels',{'-\pi','-\pi/2','0','\pi/2','\pi'}) % give them user-defined labels
    %title(ax2, sprintf("Signal Phase Robot %i to %i",i,j));

end

%[appendix]{"version":"1.0"}
%---
