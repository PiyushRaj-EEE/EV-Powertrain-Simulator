clear; clc; close all;

%% 1. Current profile (test version)
dt = 1; t = 0:dt:1800;
I  = 60*ones(size(t));          % 60 A base load
I(300:600)  = 150;              % acceleration burst
I(900:1000) = 0;                % rest period
N = length(I);

%% 2. Parameters (placeholders)
Q    = 60*3600;      % capacity [A·s]
eta  = 1;
R0   = 0.002;
R1   = 0.001;
C1   = 20000;
tau  = R1*C1;

%% 3. OCV-SOC lookup
soc_tbl = [0 0.1 0.2 0.5 0.8 0.9 1.0];
ocv_tbl = [2.8 3.20 3.28 3.30 3.33 3.38 3.45];
OCV = @(s) interp1(soc_tbl, ocv_tbl, s, 'linear', 'extrap');

%% 4. Simulation
soc = zeros(1,N); U1 = zeros(1,N); Vt = zeros(1,N);
soc(1) = 1.0;
for k = 1:N-1
    Vt(k)    = OCV(soc(k)) - I(k)*R0 - U1(k);
    soc(k+1) = soc(k) - eta*dt/Q * I(k);
    U1(k+1)  = exp(-dt/tau)*U1(k) + R1*(1-exp(-dt/tau))*I(k);
end
Vt(N) = OCV(soc(N)) - I(N)*R0 - U1(N);

%% 5. Plots
figure;
subplot(3,1,1); plot(t, I);   ylabel('Current [A]');
subplot(3,1,2); plot(t, Vt);  ylabel('Vt [V]');
subplot(3,1,3); plot(t, soc); ylabel('SOC'); xlabel('Time [s]');