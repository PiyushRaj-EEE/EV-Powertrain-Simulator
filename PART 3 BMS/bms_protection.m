clear; clc; close all;

%% 0. CHOOSE A TEST: 1 = over-temperature, 2 = under-voltage, 3 = over-voltage
scenario = 3;

%% 1. CELL PARAMETERS
Q = 60*3600; R0 = 0.002; R1 = 0.001; C1 = 20000; tau = R1*C1;
soc_tbl = [0 0.1 0.2 0.5 0.8 0.9 1.0];
ocv_tbl = [2.8 3.20 3.28 3.30 3.33 3.38 3.45];
OCV = @(s) interp1(soc_tbl, ocv_tbl, s, 'linear', 'extrap');

Cth = 600;  Rth = 1.2;  Tamb = 30;      % simple thermal model (placeholders)

%% 2. PROTECTION LIMITS
V_ov = 3.65;  V_uv = 2.80;
T_derate = 45;  T_cut = 60;
I_dis_max = 150;  I_chg_max = 100;

%% 3. TEST SCENARIO (discharge current is positive, charge is negative)
dt = 1;
switch scenario
    case 1, T_end = 1500; soc0 = 1.00; I_req_val =  120;   % hard driving
    case 2, T_end = 1500; soc0 = 0.15; I_req_val =   60;   % nearly empty
    case 3, T_end =  600; soc0 = 0.90; I_req_val =  -80;   % fast charging
end
t = 0:dt:T_end; N = length(t);
I_req = I_req_val*ones(1,N);             % what the driver / charger asks for

%% 4. SIMULATION WITH PROTECTION
soc = zeros(1,N); U1 = zeros(1,N); T = zeros(1,N);
Vt = zeros(1,N); I_ok = zeros(1,N); fault = zeros(1,N); f_der = zeros(1,N);
soc(1) = soc0; T(1) = Tamb;
I_prev = 0; flt = 0;

for k = 1:N
    % (a) what the BMS measures right now
    V_meas = OCV(soc(k)) - I_prev*R0 - U1(k);

    % (b) check for faults (latched: once tripped, it stays tripped)
    if flt == 0
        if     V_meas > V_ov,  flt = 1;      % over-voltage
        elseif V_meas < V_uv,  flt = 2;      % under-voltage
        elseif T(k) >= T_cut,  flt = 3;      % over-temperature
        end
    end

    % (c) temperature derating factor (1 = full power, 0 = none)
    f = min(max((T_cut - T(k)) / (T_cut - T_derate), 0), 1);

    % (d) allowed current = requested current, limited by the rules
    I_ok(k) = min(max(I_req(k), -I_chg_max*f), I_dis_max*f);
    if flt > 0, I_ok(k) = 0; end             % fault: open the contactor

    % (e) record
    Vt(k) = OCV(soc(k)) - I_ok(k)*R0 - U1(k);
    fault(k) = flt;  f_der(k) = f;

    % (f) update the battery state
    if k < N
        P = I_ok(k)^2*R0 + U1(k)^2/R1;                       % heat [W]
        T(k+1)   = T(k) + dt/Cth*(P - (T(k)-Tamb)/Rth);
        soc(k+1) = soc(k) - dt/Q*I_ok(k);
        U1(k+1)  = exp(-dt/tau)*U1(k) + R1*(1-exp(-dt/tau))*I_ok(k);
    end
    I_prev = I_ok(k);
end

%% 5. REPORT AND PLOTS
names = {'None','Over-voltage','Under-voltage','Over-temperature'};
i1 = find(fault > 0, 1);
if isempty(i1)
    fprintf('No fault. Max temperature = %.1f C\n', max(T));
else
    fprintf('FAULT: %s at t = %d s (SOC = %.3f)\n', names{fault(i1)+1}, t(i1), soc(i1));
end

figure('Position',[100 100 900 800]);
subplot(4,1,1);
plot(t, I_req, '--', 'Color', [1 0.6 0.1], 'LineWidth', 2); hold on;
plot(t, I_ok, 'Color', [0.1 0.7 1], 'LineWidth', 2); grid on;
ylabel('Current [A]'); legend('Requested','Allowed by BMS');
title(sprintf('BMS protection test, scenario %d', scenario));

subplot(4,1,2);
plot(t, Vt, 'Color', [0.3 0.9 0.3], 'LineWidth', 2); hold on;
yline(V_ov, '--r', 'Over-voltage'); yline(V_uv, '--r', 'Under-voltage');
grid on; ylabel('Cell voltage [V]');

subplot(4,1,3);
plot(t, T, 'Color', [1 0.4 0.4], 'LineWidth', 2); hold on;
yline(T_derate, '--y', 'Derate starts'); yline(T_cut, '--r', 'Hard cut');
grid on; ylabel('Temperature [C]');

subplot(4,1,4);
stairs(t, fault, 'Color', [0.8 0.5 1], 'LineWidth', 2); grid on;
ylim([-0.2 3.5]); yticks(0:3);
yticklabels({'OK','Over-V','Under-V','Over-T'});
ylabel('Fault'); xlabel('Time [s]');