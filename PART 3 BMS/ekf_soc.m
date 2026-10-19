clear; clc; close all; rng(1);

%% 1. TRUE BATTERY (same as step 1, this is the "real world")
dt = 1; t = 0:dt:1800; N = length(t);
I = 60*ones(1,N); I(301:600) = 150; I(901:1000) = 0;

Q = 60*3600; eta = 1;
R0 = 0.002; R1 = 0.001; C1 = 20000; tau = R1*C1;

soc_tbl = [0 0.1 0.2 0.5 0.8 0.9 1.0];
ocv_tbl = [2.8 3.20 3.28 3.30 3.33 3.38 3.45];
OCV = @(s) interp1(soc_tbl, ocv_tbl, s, 'linear', 'extrap');

soc = zeros(1,N); U1 = zeros(1,N); Vt = zeros(1,N);
soc(1) = 0.8;
for k = 1:N-1
    Vt(k)    = OCV(soc(k)) - I(k)*R0 - U1(k);
    soc(k+1) = soc(k) - eta*dt/Q*I(k);
    U1(k+1)  = exp(-dt/tau)*U1(k) + R1*(1-exp(-dt/tau))*I(k);
end
Vt(N) = OCV(soc(N)) - I(N)*R0 - U1(N);

%% 2. SENSORS (what the BMS actually sees)
Vm = Vt + 0.005*randn(1,N);          % voltage sensor: 5 mV noise
Im = 1.02*I + 0.5*randn(1,N);        % current sensor: +2% bias, 0.5 A noise

%% 3. EKF SETUP
x  = [0.6; 0];                       % initial guess: SOC = 0.8 (truth is 1.0!)
P  = diag([0.05, 1e-4]);             % initial uncertainty
Qn = diag([1e-9, 1e-7]);             % model noise (distrust of model)
Rn = 0.005^2;                        % sensor noise (distrust of voltage sensor)

A = [1 0; 0 exp(-dt/tau)];
B = [-eta*dt/Q; R1*(1-exp(-dt/tau))];

soc_ekf = zeros(1,N); soc_cc = zeros(1,N);
soc_ekf(1) = x(1);  soc_cc(1) = 0.6; % Coulomb counting gets the same wrong start

%% 4. EKF LOOP
for k = 2:N
    % --- PREDICT ---
    x = A*x + B*Im(k-1);
    P = A*P*A' + Qn;
    soc_cc(k) = soc_cc(k-1) - eta*dt/Q*Im(k-1);   % plain Coulomb counting

    % --- CORRECT ---
    ds   = 1e-3;
    dOCV = (OCV(x(1)+ds) - OCV(x(1)-ds)) / (2*ds); % slope of OCV curve
    H    = [dOCV, -1];
    Vt_pred = OCV(x(1)) - Im(k)*R0 - x(2);
    innov   = Vm(k) - Vt_pred;

    S = H*P*H' + Rn;
    K = P*H'/S;
    x = x + K*innov;
    x(1) = min(max(x(1), 0), 1);                   % keep SOC between 0 and 1
    P = (eye(2) - K*H)*P;

    soc_ekf(k) = x(1);
end

%% 5. RESULTS
rmse_ekf = sqrt(mean((soc_ekf - soc).^2));
rmse_cc  = sqrt(mean((soc_cc  - soc).^2));
fprintf('RMSE  EKF: %.4f   Coulomb counting: %.4f\n', rmse_ekf, rmse_cc);

figure;
subplot(2,1,1);
plot(t, soc, 'k', t, soc_cc, 'r--', t, soc_ekf, 'b'); grid on;
legend('True SOC','Coulomb counting','EKF'); ylabel('SOC');
subplot(2,1,2);
plot(t, soc_cc - soc, 'r--', t, soc_ekf - soc, 'b'); grid on;
legend('CC error','EKF error'); ylabel('SOC error'); xlabel('Time [s]');