clear; close all; clc

% 1. Laad de gecombineerde dataset
load('results/circletest.mat');

% scale factor figure text
sfactor = 4;

% 3. Bepaal de dimensies voor de plots
Nmov    = size(mc_p_error_odom{1}, 1);   % Aantal robots
Nstat   = size(mc_AoA_error_odom{1}, 2); % Aantal ankers
mc_itrs = length(mc_p_error_odom);       % Totaal aantal gecombineerde runs

% 4. Zet de cell arrays om naar handige 3D matrices
p_err_3d   = cat(3, mc_p_error_odom{:});     
AoA_err_3d = cat(3, mc_AoA_error_odom{:}); 
p_est_3d   = cat(3, mc_p_est_odom{:});       
p_true_3d  = cat(3, mc_p_true{:});     

% Kleuren voor de robots
colors = [0.00, 0.45, 0.74;  
          0.85, 0.33, 0.10]; 

% =========================================================================
% FIGURE 1: 2D POSITION SPREAD (Schatting vs Werkelijkheid)
% =========================================================================
figure(1); clf; hold on;
for r = 1:Nmov
    x_est = squeeze(p_est_3d(r, 1, :));
    y_est = squeeze(p_est_3d(r, 2, :));
    x_true = p_true_3d(r, 1, 1); 
    y_true = p_true_3d(r, 2, 1);
%     if r==1
%         x_true = 0.18;
%         y_true = 2.12;
%     else
%         x_true =1;
%         y_true=-1;
%     end
    
    % Jitter toevoegen zodat overlappende punten zichtbaar worden
    jx = (rand(size(x_est)) - 0.5) * 0.000004; 
    jy = (rand(size(y_est)) - 0.5) * 0.000004;
    
    % Teken lijntjes (error vectors) van de ware positie naar de schattingen
    for i = 1:mc_itrs
        plot([x_true, x_est(i)+jx(i)], [y_true, y_est(i)+jy(i)], ...
             '--', 'Color', [colors(r,:) 0.4], 'HandleVisibility', 'off');
    end
    
    % Plot schattingen (Met zwarte rand voor beter contrast)
    scatter(x_est + jx, y_est + jy, 80, colors(r,:), 'filled', ...
        'MarkerEdgeColor', 'k', 'MarkerFaceAlpha', 0.8, ...
        'DisplayName', sprintf('Robot %d (Est)', r));
    
    % Plot ware positie (Als een grote 'Ster' (pentagram))
    scatter(x_true, y_true, 400, colors(r,:), 'p', 'filled', ...
        'MarkerEdgeColor', 'k', 'LineWidth', 1.5, ...
        'DisplayName', sprintf('Robot %d (True)', r));
end
title('Monte Carlo 2D Position Spread');
xlabel('X Position [m]'); ylabel('Y Position [m]');
legend('Location', 'best', 'FontSize', 11);
set(gca, 'FontSize', 12, 'GridAlpha', 0.3);
grid on; grid minor; axis equal;
fontsize(scale=sfactor)


% =========================================================================
% FIGURE 2: POSITION ERROR (Boxplot + Swarm Overlay + CDF)
% =========================================================================
figure(2); clf; 
p_err_matrix = squeeze(p_err_3d)'; 

% Subplot 2A: Boxplot met Swarm overlay
subplot(1, 2, 1); hold on;
boxplot(p_err_matrix, 'Labels', {'Robot 1', 'Robot 2'}, 'Colors', 'k', 'Symbol', '');
for r = 1:Nmov
    % Lichte X-jitter zodat de bolletjes niet perfect op 1 verticale lijn liggen
    x_swarm = r + (rand(mc_itrs, 1) - 0.5) * 0.1; 
    scatter(x_swarm, p_err_matrix(:, r), 70, colors(r,:), 'filled', ...
            'MarkerEdgeColor', 'k', 'MarkerFaceAlpha', 0.6);
end
title('Position Error Distribution (Box + Swarm)');
ylabel('Position Error [m]');
set(gca, 'FontSize', 12);
grid on; grid minor;

% Subplot 2B: CDF (Cumulative Distribution Function)
subplot(1, 2, 2); hold on;
for r = 1:Nmov
    [f, x_cdf] = ecdf(p_err_matrix(:, r));
    plot(x_cdf, f, 'Color', colors(r,:), 'LineWidth', 2.5, 'DisplayName', sprintf('Robot %d', r));
end
title('CDF of Position Error');
xlabel('Position Error [m]'); ylabel('Probability');
legend('Location', 'southeast', 'FontSize', 11);
set(gca, 'FontSize', 12);
grid on; grid minor;
fontsize(scale=sfactor)


% =========================================================================
% FIGURE 3: AoA ERROR ANALYSIS (Overlapping Bell Curves met Mu & Sigma)
% =========================================================================
figure(3); 
set(gcf, 'Position', [100, 100, 700, 400]); 
clf; hold on;

% Data splitsen op basis van LOS en NLOS (Ankers 2&3 LOS, 1&4 NLOS)
AoA_LOS  = AoA_err_3d(:, [2, 3], :);
AoA_NLOS = AoA_err_3d(:, [1, 4], :);

AoA_LOS_flat  = AoA_LOS(:);
AoA_NLOS_flat = AoA_NLOS(:);

% Bereken Gemiddelde (mu) en Standaarddeviatie (sigma)
mu_LOS    = mean(AoA_LOS_flat);
sigma_LOS = std(AoA_LOS_flat);
mu_NLOS    = mean(AoA_NLOS_flat);
sigma_NLOS = std(AoA_NLOS_flat);

% Maak een X-as (range) voor de bell curves
x_min = min([AoA_LOS_flat; AoA_NLOS_flat]) - 2;
x_max = max([AoA_LOS_flat; AoA_NLOS_flat]) + 2;
x_pdf = linspace(x_min, x_max, 1000); 

% Bereken de Normal Probability Density Functions (bell curves)
y_LOS  = normpdf(x_pdf, mu_LOS, sigma_LOS);
y_NLOS = normpdf(x_pdf, mu_NLOS, sigma_NLOS);

color_LOS  = [0.4660, 0.6740, 0.1880]; % Groen voor LOS
color_NLOS = [0.6350, 0.0780, 0.1840]; % Rood voor NLOS

% Histogrammen en Bell curves plotten
histogram(AoA_LOS_flat, 'Normalization', 'pdf', 'FaceColor', color_LOS, 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'HandleVisibility', 'off');
histogram(AoA_NLOS_flat, 'Normalization', 'pdf', 'FaceColor', color_NLOS, 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'HandleVisibility', 'off');

a1 = area(x_pdf, y_LOS, 'FaceColor', color_LOS, 'FaceAlpha', 0.5, 'EdgeColor', color_LOS, 'LineWidth', 2.5);
a2 = area(x_pdf, y_NLOS, 'FaceColor', color_NLOS, 'FaceAlpha', 0.5, 'EdgeColor', color_NLOS, 'LineWidth', 2.5);

xline(mu_LOS, '--', 'Color', color_LOS, 'LineWidth', 1.5, 'HandleVisibility', 'off');
xline(mu_NLOS, '--', 'Color', color_NLOS, 'LineWidth', 1.5, 'HandleVisibility', 'off');

% Voeg tekstlabels toe voor mu en sigma
offset_x = (x_max - x_min) * 0.05; 
text(mu_LOS + offset_x, max(y_LOS) * 0.75, ...
    sprintf('mean = %.2f^\\circ\nstdev = %.2f^\\circ', mu_LOS, sigma_LOS), ...
    'Color', color_LOS, 'FontSize', 12, 'FontWeight', 'bold', 'Interpreter', 'tex');
text(mu_NLOS + offset_x, max(y_NLOS) * 0.75, ...
    sprintf('mean = %.2f^\\circ\nstdev = %.2f^\\circ', mu_NLOS, sigma_NLOS), ...
    'Color', color_NLOS, 'FontSize', 12, 'FontWeight', 'bold', 'Interpreter', 'tex');

title('AoA Error Distribution: LOS vs. NLOS');
xlabel('Angle of Arrival Error [Degrees]');
ylabel('Probability Density');
legend([a1, a2], {'LOS (Anchors 2 & 3)', 'NLOS (Anchors 1 & 4)'}, 'Location', 'northeast', 'FontSize', 11);
set(gca, 'FontSize', 12);
grid on; grid minor;
fontsize(scale=sfactor)

% =========================================================================
% FIGURE 4: AoA ERROR PER ANCHOR
% =========================================================================

figure(4); clf;
AoA_per_anchor = reshape(squeeze(AoA_err_3d),8,[],1); 
boxplot(AoA_per_anchor')%, 'Labels', {'Anchor 1', 'Anchor 2', 'Anchor 3', 'Anchor 4'});
title('AoA Error per Static Anchor');
ylabel('AoA Error [Degrees]');
xlabel('Static Anchor Index');
set(gca, 'FontSize', 12);
grid on;
avg_sys_aoa = mean(AoA_per_anchor(:));
yline(avg_sys_aoa, 'r--', 'System Avg', 'LineWidth', 1.5, 'FontSize', 11);
fontsize(scale=sfactor)

% =========================================================================
% DATAPUNTEN VERIFICATIE (Print in Command Window)
% =========================================================================
fprintf('\n--- VERIFICATIE VAN DATAPUNTEN ---\n');
fprintf('Totaal aantal gefilterde Monte Carlo runs: %d\n\n', mc_itrs);
for r = 1:Nmov
    fprintf('Robot %d:\n', r);
    fprintf('  - Aantal datapunten: %d\n', length(squeeze(p_est_3d(r, 1, :))));
end
fprintf('----------------------------------\n\n');


% =========================================================================

% FIGURE 5: DRIFT RMSE vs. POSITION ERROR (CORRELATIE PLOT)

% =========================================================================

% Controleer of de drift data bestaat in deze dataset
if exist('mc_drift_rmse', 'var') && exist('ruwe_p_error', 'var')

% Zorg dat de lege indices (valid_idx uit het begin van je script)
% ook worden toegepast op de drift, zodat arrays even lang blijven
mc_drift_rmse_filtered = mc_drift_rmse(valid_idx);

% Converteer de cel-lijsten (2x1 per cel) naar 3D en daarna naar een vlakke matrix [N_runs x 2]
drift_rmse_mat = squeeze(cat(3, mc_drift_rmse_filtered{:}))';
% We gebruiken p_err_matrix die je al berekend had in Figure 2
% (of we berekenen hem even opnieuw voor de zekerheid)
p_err_mat = squeeze(cat(3, mc_p_error{:}))';
figure(5); clf; hold on;

% Teken de punten voor Robot 1 en Robot 2
for r = 1:Nmov
scatter(drift_rmse_mat(:, r), p_err_mat(:, r), 80, colors(r,:), 'filled', 'MarkerEdgeColor', 'k', 'MarkerFaceAlpha', 0.8, 'DisplayName', sprintf('Robot %d', r));
% Optioneel: Fit een lineaire trendlijn per robot om de correlatie te laten zien
% (Haal de '%' weg als je een trendlijn wilt zien)
% p = polyfit(drift_rmse_mat(:, r), p_err_mat(:, r), 1);
% x_trend = linspace(min(drift_rmse_mat(:,r)), max(drift_rmse_mat(:,r)), 100);
% y_trend = polyval(p, x_trend);
% plot(x_trend, y_trend, '--', 'Color', colors(r,:), 'LineWidth', 1.5, 'HandleVisibility', 'off');
end
title('Impact of Odometry Drift on Final Position Error');
xlabel('Odometry Drift RMSE [m]');
ylabel('Final Position Error [m]');
legend('Location', 'best', 'FontSize', 11);
set(gca, 'FontSize', 12);
grid on; grid minor;
fontsize(scale=sfactor)

else
warning('mc_drift_rmse is niet gevonden. Run eerst je geüpdatete combiner-script!')
end

% % =========================================================================
% % FIGURE 5: INCREMENTAL DRIFT RMSE vs. POSITION ERROR (CORRELATIE PLOT)
% % =========================================================================
% % Controleer of de nieuwe afgeleide drift data bestaat in deze dataset
% if exist('mc_drift_diff_rmse', 'var') && exist('ruwe_p_error', 'var')
%     
%     % Toepassen van de lege-index filter
%     mc_drift_diff_filtered = mc_drift_diff_rmse(valid_idx);
%     
%     % Converteer de cel-lijsten naar een vlakke matrix [N_runs x 2]
%     drift_rmse_mat = squeeze(cat(3, mc_drift_diff_filtered{:}))';
%     
%     % Final position error matrix
%     p_err_mat = squeeze(cat(3, mc_p_error{:}))'; 
% 
%     figure(5); clf; hold on;
%     
%     % Teken de punten voor Robot 1 en Robot 2
%     for r = 1:Nmov
%         scatter(drift_rmse_mat(:, r), p_err_mat(:, r), 80, colors(r,:), ...
%             'filled', 'MarkerEdgeColor', 'k', 'MarkerFaceAlpha', 0.8, ...
%             'DisplayName', sprintf('Robot %d', r));
%             
%         % Optioneel: Fit een lineaire trendlijn
%         % p = polyfit(drift_rmse_mat(:, r), p_err_mat(:, r), 1);
%         % x_trend = linspace(min(drift_rmse_mat(:,r)), max(drift_rmse_mat(:,r)), 100);
%         % y_trend = polyval(p, x_trend);
%         % plot(x_trend, y_trend, '--', 'Color', colors(r,:), 'LineWidth', 1.5, 'HandleVisibility', 'off');
%     end
% 
%     title('Impact of Incremental Drift (Rate of Change) on Final Position Error');
%     xlabel('Incremental Odometry Drift RMSE [m / step]');
%     ylabel('Final Position Error [m]');
%     legend('Location', 'best', 'FontSize', 11);
%     set(gca, 'FontSize', 12);
%     grid on; grid minor;
%     
% else
%     warning('mc_drift_diff_rmse is niet gevonden. Run eerst je geüpdatete combiner-script!');
% end

% =========================================================================
% FIGURE 6: AoA ERROR DENSITY CURVES PER RECEIVER-TRANSMITTER PAIR
% =========================================================================
figure(6); 
set(gcf, 'Position', [100, 100, 1200, 800]); 
clf; 

% Maak subplot grid: 2 receivers x 4 transmitters
nrows = 2; ncols = 4;
colors_rec = [0.4660, 0.6740, 0.1880]; % Groen-achtig voor Receiver 1
colors_tr  = [0.6350, 0.0780, 0.1840; 0.00, 0.45, 0.74; 0.85, 0.33, 0.10; 0.4940, 0.1840, 0.5560];

for r = 1:Nmov  % Receivers (robots)
    for t = 1:Nstat  % Transmitters (anchors)
        subplot(nrows, ncols, (r-1)*ncols + t);
        hold on;
        
        % Extract data voor deze specifieke receiver-transmitter pair
        aoa_pair = squeeze(AoA_err_3d(r, t, :));  % [1 x mc_itrs]
        
        % Filter NaN/Inf values (voor de zekerheid)
        aoa_valid = aoa_pair(~isnan(aoa_pair) & ~isinf(aoa_pair));
        
        if isempty(aoa_valid)
            text(0.5, 0.5, 'No valid data', 'HorizontalAlignment', 'center', ...
                 'VerticalAlignment', 'middle', 'FontSize', 12);
            title(sprintf('R%d-T%d', r, t));
            continue;
        end
        
        % Bereken statistieken
        mu_pair = mean(aoa_valid);
        sigma_pair = std(aoa_valid);
        
        % Maak X-as range voor bell curve
        x_min = min(aoa_valid) - 2*sigma_pair;
        x_max = max(aoa_valid) + 2*sigma_pair;
        x_pdf = linspace(x_min, x_max, 1000);
        
        % Normal PDF
        y_pdf = normpdf(x_pdf, mu_pair, sigma_pair);
        
        % Kleur per transmitter (consistent over receivers)
        color_pair = colors_tr(t, :);
        
        % Histogram (transparant achtergrond)
        histogram(aoa_valid, 'Normalization', 'pdf', ...
                  'FaceColor', color_pair, 'FaceAlpha', 0.15, ...
                  'EdgeColor', 'none', 'HandleVisibility', 'off');
        
        % Bell curve (area fill zoals Figure 3)
        area_fill = area(x_pdf, y_pdf, 'FaceColor', color_pair, ...
                        'FaceAlpha', 0.5, 'EdgeColor', color_pair, ...
                        'LineWidth', 2);
        
        % Mean line
        xline(mu_pair, '--', 'Color', color_pair, 'LineWidth', 1.5, ...
              'HandleVisibility', 'off');
        
        % Tekstlabel met mean en std
        offset_x = (x_max - x_min) * 0.05;
        text(mu_pair + offset_x, max(y_pdf) * 0.75, ...
             sprintf('μ=%.2f°\nσ=%.2f°', mu_pair, sigma_pair), ...
             'Color', color_pair, 'FontSize', 11, 'FontWeight', 'bold', ...
             'Interpreter', 'tex', 'HorizontalAlignment', 'left');
        
        title(sprintf('R%d → T%d', r, t), 'FontSize', 12, 'FontWeight', 'bold');
        xlabel('AoA Error [°]');
        ylabel('Density');
        
        % Grid en styling
        grid on; grid minor;
        set(gca, 'FontSize', 11);
        
        % Gelijke aspect ratio voor consistentie
        xlim([x_min, x_max]);
        ylim([0, max(y_pdf)*1.2]);
    end
end

% Global title
sgtitle('AoA Error Density: Individual Receiver-Transmitter Pairs', ...
        'FontSize', 16, 'FontWeight', 'bold');

% Apply fontsize scaling
fontsize(scale=sfactor);

% Optioneel: Print statistieken naar command window
fprintf('\n--- AoA ERROR STATISTICS PER R-T PAIR ---\n');
for r = 1:Nmov
    fprintf('Receiver %d:\n', r);
    for t = 1:Nstat
        aoa_pair = squeeze(AoA_err_3d(r, t, :));
        aoa_valid = aoa_pair(~isnan(aoa_pair) & ~isinf(aoa_pair));
        if ~isempty(aoa_valid)
            mu = mean(aoa_valid); sigma = std(aoa_valid);
            fprintf('  T%d: μ=%.2f°, σ=%.2f° (N=%d)\n', t, mu, sigma, length(aoa_valid));
        end
    end
end
fprintf('---------------------------------------\n');