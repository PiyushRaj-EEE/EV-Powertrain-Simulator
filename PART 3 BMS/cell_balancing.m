clear; clc; close all;

%% 1. PACK SETUP
nCells = 4;
Q   = 10*3600*ones(nCells,1);     % capacity per cell [A·s] (10 Ah demo cells)
soc = [0.98; 0.92; 0.95; 0.90];   % unbalanced starting SOC
dt  = 10;                         % time step [s]
t   = 0:dt:4*3600;                % 4 hours of resting/balancing
N   = length(t);

soc_tbl = [0 0.1 0.2 0.5 0.8 0.9 1.0];
ocv_tbl = [2.8 3.20 3.28 3.30 3.33 3.38 3.45];
OCV = @(s) interp1(soc_tbl, ocv_tbl, s, 'linear', 'extrap');

%% 2. BALANCING SETTINGS
Rb    = 10;       % bleed resistor [ohm]
dV_on = 0.005;    % bleed a cell if it is 5 mV above the lowest cell

%% 3. SIMULATION
SOC   = zeros(nCells, N);
V     = zeros(nCells, N);
bleed = zeros(nCells, N);
SOC(:,1) = soc;

for k = 1:N
    V(:,k)     = OCV(SOC(:,k));            % at rest, voltage = OCV
    Vmin       = min(V(:,k));              % lowest cell is the reference
    bleed(:,k) = (V(:,k) - Vmin) > dV_on;  % 1 = bleeding ON for that cell
    if k < N
        Ib = bleed(:,k) .* V(:,k) / Rb;            % bleed current [A]
        SOC(:,k+1) = SOC(:,k) - Ib*dt ./ Q;        % remove charge
    end
end

%% 4. RESULTS
spread0 = (max(SOC(:,1))   - min(SOC(:,1)))   * 100;
spreadF = (max(SOC(:,end)) - min(SOC(:,end))) * 100;
fprintf('SOC spread: %.1f %% -> %.1f %%\n', spread0, spreadF);
fprintf('Voltage spread: %.0f mV -> %.0f mV\n', ...
    (max(V(:,1))-min(V(:,1)))*1000, (max(V(:,end))-min(V(:,end)))*1000);

th = t/3600;
c = lines(nCells);
figure('Position',[100 100 900 750]);

subplot(3,1,1); hold on;
for i = 1:nCells, plot(th, SOC(i,:), 'Color', c(i,:), 'LineWidth', 2); end
grid on; ylabel('SOC'); title('Passive cell balancing (4 cells in series)');
legend('Cell 1','Cell 2','Cell 3','Cell 4');

subplot(3,1,2); hold on;
for i = 1:nCells, plot(th, V(i,:), 'Color', c(i,:), 'LineWidth', 2); end
grid on; ylabel('Cell voltage [V]');

subplot(3,1,3); hold on;
for i = 1:nCells
    stairs(th, bleed(i,:) + 1.5*(i-1), 'Color', c(i,:), 'LineWidth', 1.5);
end
grid on; ylabel('Bleed ON (per cell)'); xlabel('Time [hours]');
yticks(1.5*(0:nCells-1)); yticklabels({'C1','C2','C3','C4'});