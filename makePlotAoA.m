function figAoA = makePlotAoA(AOA_profile, betaList,gammaList, plotVisible, true_azimuth, true_elevation)

        figAoA = figure("Visible",plotVisible,"Position",[909 320 560 200],"Theme","Light");
        
        betaDeg = rad2deg(betaList);
        betaMin = betaDeg(1);
        betaMax = betaDeg(end);
        betaNum = numel(betaDeg);
        gammaDeg = rad2deg(gammaList);
        gammaMin = gammaDeg(1);
        gammaMax = gammaDeg(end);
        gammaNum = numel(gammaDeg);


        % % Side view plot (tile 1)
        % ax1 = nexttile; % Get the axes for the first tile
        % surf(ax1, rad2deg(betaList), rad2deg(gammaList), AOA_profile.', 'EdgeColor', 'none');
        % set(gcf,'Renderer','Zbuffer');            
        % xlabel(ax1, 'Azimuth (degrees)');
        % ylabel(ax1, 'Elevation (degrees)');
        % zlabel(ax1, 'Spectrum (-)');
        % xlim(ax1, [-180,180])
        % ylim(ax1, [0,180])
        % zlim(ax1, [0 max(max(AOA_profile))]);
        % title(ax1, sprintf('AOA Profile Robot %i to %i (Side View)',i,j));

        
        % Make sure azimuth is in right format
        true_azimuth = wrapTo180(true_azimuth);
        
        % Top view plot (tile 2)
        ax2 = nexttile; % Get the axes for the second tile

        % Add crosshair to surf
        %drawcrosshair('Parent',ax2,'Position',[wrapTo180(true_azimuth + 180) true_elevation + 90],'LineWidth',2,'Color','r'); % doesnt work nice
        pad = 5;
        z_val = max(AOA_profile,[],'all');
        
        % find two closest cadidate indeces for the true azimuth
        cx1 = find(true_azimuth>=betaDeg,1,"last");
        cx2 = find(true_azimuth<=betaDeg,1,"first");
        % Select the closest
        if abs(true_azimuth-betaDeg(cx1)) < abs(true_azimuth-betaDeg(cx2))
            cx = cx1;
        else
            cx = cx2;
        end
        % Make the line with padding
        cx_left = 1:max(1,cx-pad);
        cx_right = min(betaNum,cx+pad):betaNum;

        % find two closest cadidate indeces for the true elevation
        cy1 = find(true_elevation>=gammaDeg,1,"last");
        cy2 = find(true_elevation<=gammaDeg,1,"first");
        % Select the closest
        if abs(true_elevation-gammaDeg(cy1)) < abs(true_elevation-gammaDeg(cy2))
            cy = cy1;
        else
            cy = cy2;
        end
        % Make the line with padding
        cy_top = 1:max(1,cy-pad);
        cy_bottom = min(gammaNum,cy+pad):gammaNum;

        % Edit the AoA plot data to add crosshair
        AOA_profile([cx_left,cx_right],cy) = z_val;
        AOA_profile(cx,[cy_top,cy_bottom]) = z_val;


        % Plot
        surf(ax2, betaDeg, gammaDeg, AOA_profile.', 'EdgeColor', 'none');
        set(gcf,'Renderer','Zbuffer');
        view(ax2, 2);
        xlabel(ax2, 'Azimuth (degrees)');
        ylabel(ax2, 'Elevation (degrees)');
        zlabel(ax2, 'Estimated spectrum (-)');
        xlim(ax2, [betaMin,betaMax]);
        ylim(ax2, [gammaMin,gammaMax]);
        xticks(betaMin:90:betaMax);
        yticks(gammaMin:90:gammaMax);
        colorbar;
    end

%[appendix]{"version":"1.0"}
%---
