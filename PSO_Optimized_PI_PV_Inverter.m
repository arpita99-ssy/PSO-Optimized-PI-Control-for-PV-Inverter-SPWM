%% ============================================================
% PSO OPTIMIZED PI CONTROL FOR PV INVERTER USING SPWM
% MATLAB R2025b
% =============================================================

clear;
clc;
close all;

%% ============================================================
% 1. SIMULATION PARAMETERS
% ============================================================

Fs = 20000;                 % Sampling frequency
Ts = 1/Fs;                  % Sampling time
Tsim = 0.20;                % Simulation time
t = 0:Ts:Tsim;
N = length(t);

fout = 50;                  % AC output frequency
w = 2*pi*fout;

Vdc = 400;                  % DC-link voltage
Vrms_ref = 230;             % Desired RMS voltage
Vpeak_ref = sqrt(2)*Vrms_ref;

Rload = 50;                 % Load resistance

%% ============================================================
% 2. LC FILTER PARAMETERS
% ============================================================

Lf = 5e-3;                  % Filter inductance
Rf = 0.15;                  % Inductor resistance
Cf = 20e-6;                 % Filter capacitance

%% ============================================================
% 3. PV ARRAY
% ============================================================

Ppv_rated = 1000;           % Rated PV power
Vmp = 250;                  % Voltage at maximum power
Imp = Ppv_rated/Vmp;        % Current at maximum power

G = 1000;                   % Irradiance
Tcell = 25;                 % Cell temperature

Vpv = Vmp;
Ipv = Imp*(G/1000);
Ppv = Vpv*Ipv;

fprintf('\n====================================\n');
fprintf('PV ARRAY PARAMETERS\n');
fprintf('====================================\n');
fprintf('Irradiance       : %.0f W/m^2\n',G);
fprintf('Temperature      : %.1f deg C\n',Tcell);
fprintf('PV Voltage       : %.2f V\n',Vpv);
fprintf('PV Current       : %.2f A\n',Ipv);
fprintf('PV Power         : %.2f W\n',Ppv);

%% ============================================================
% 4. MPPT REFERENCE
% ============================================================

% Simplified MPPT operating point
Vmppt = Vmp*ones(1,N);

% Small irradiance variation for demonstration
G_profile = 1000*ones(1,N);

G_profile(t >= 0.08 & t < 0.14) = 900;
G_profile(t >= 0.14) = 1000;

Pmppt = Ppv_rated*(G_profile/1000);

%% ============================================================
% 5. REFERENCE AC VOLTAGE
% ============================================================

Vref = Vpeak_ref*sin(w*t);

%% ============================================================
% 6. PSO PARAMETERS
% ============================================================

nParticles = 25;
nIterations = 35;

% PI gain limits
Kp_min = 0.1;
Kp_max = 10;

Ki_min = 10;
Ki_max = 2000;

% PSO constants
w_pso = 0.72;
c1 = 1.5;
c2 = 1.5;

%% ============================================================
% 7. INITIALIZE PARTICLES
% ============================================================

particle = zeros(nParticles,2);
velocity = zeros(nParticles,2);

pBest = zeros(nParticles,2);
pBestCost = inf(nParticles,1);

gBest = [1 100];
gBestCost = inf;

for i = 1:nParticles

    particle(i,1) = ...
        Kp_min + rand*(Kp_max-Kp_min);

    particle(i,2) = ...
        Ki_min + rand*(Ki_max-Ki_min);

    pBest(i,:) = particle(i,:);

end

%% ============================================================
% 8. PSO OPTIMIZATION
% ============================================================

fprintf('\n====================================\n');
fprintf('STARTING PSO OPTIMIZATION\n');
fprintf('====================================\n');

for iter = 1:nIterations

    for p = 1:nParticles

        Kp = particle(p,1);
        Ki = particle(p,2);

        cost = inverterObjective( ...
            Kp,Ki,Ts,N,Vdc,Vref,...
            Lf,Rf,Cf,Rload);

        % Personal best
        if cost < pBestCost(p)

            pBestCost(p) = cost;
            pBest(p,:) = particle(p,:);

        end

        % Global best
        if cost < gBestCost

            gBestCost = cost;
            gBest = particle(p,:);

        end

    end

    % Update particles
    for p = 1:nParticles

        r1 = rand;
        r2 = rand;

        velocity(p,:) = ...
            w_pso*velocity(p,:) ...
            + c1*r1*(pBest(p,:)-particle(p,:)) ...
            + c2*r2*(gBest-particle(p,:));

        particle(p,:) = particle(p,:) + velocity(p,:);

        % Apply limits
        particle(p,1) = ...
            max(min(particle(p,1),Kp_max),Kp_min);

        particle(p,2) = ...
            max(min(particle(p,2),Ki_max),Ki_min);

    end

    fprintf('Iteration %2d/%2d : Kp = %.5f | Ki = %.5f | Cost = %.6f\n',...
        iter,nIterations,gBest(1),gBest(2),gBestCost);

end

Kp_opt = gBest(1);
Ki_opt = gBest(2);

fprintf('\n====================================\n');
fprintf('OPTIMIZED PI GAINS\n');
fprintf('====================================\n');
fprintf('Kp = %.6f\n',Kp_opt);
fprintf('Ki = %.6f\n',Ki_opt);

%% ============================================================
% 9. FINAL INVERTER SIMULATION
% ============================================================

Kp = Kp_opt;
Ki = Ki_opt;

% Initialize states
iL = 0;
Vc = 0;
integral = 0;

% Arrays
Vout = zeros(1,N);
Iout = zeros(1,N);
IL = zeros(1,N);
Vinv = zeros(1,N);
control = zeros(1,N);
error = zeros(1,N);
Pout = zeros(1,N);

% Integral limit
integral_limit = 300;

%% ============================================================
% 10. CLOSED LOOP SIMULATION
% ============================================================

for k = 2:N

    % Reference
    ref = Vref(k);

    % Feedback voltage
    feedback = Vc;

    % Error
    error(k) = ref-feedback;

    % PI integral
    integral = integral + error(k)*Ts;

    % Anti-windup
    integral = max(min(integral,integral_limit),-integral_limit);

    % PI controller
    u = Kp*error(k) + Ki*integral;

    % Modulation index
    m = u/Vdc;

    % Saturation
    m = max(min(m,0.95),-0.95);

    control(k) = m;

    % Averaged inverter voltage
    Vinv(k) = m*Vdc;

    % ========================================================
    % LC FILTER MODEL
    % ========================================================

    % Inductor current derivative
    diL = (Vinv(k) - Rf*iL - Vc)/Lf;

    % Capacitor voltage derivative
    dVc = (iL - Vc/Rload)/Cf;

    % Euler integration
    iL = iL + diL*Ts;

    Vc = Vc + dVc*Ts;

    % Store values
    IL(k) = iL;
    Vout(k) = Vc;

    % Load current
    Iout(k) = Vc/Rload;

    % Output power
    Pout(k) = Vc*Iout(k);

end

%% ============================================================
% 11. GENERATE REAL SPWM
% ============================================================

fpwm = 5000;

% Triangular carrier from -1 to +1
carrier = 2*abs(2*(fpwm*t-floor(fpwm*t+0.5)))-1;

% Modulating signal
modulating = control;

% SPWM pulses
PWM = modulating >= carrier;

%% ============================================================
% 12. RMS CALCULATIONS
% ============================================================

% Ignore initial transient
startIndex = round(0.05/Ts);

Vout_rms = rms(Vout(startIndex:end));
Iout_rms = rms(Iout(startIndex:end));

Pout_avg = mean(Pout(startIndex:end));

Efficiency = ...
    (Pout_avg/mean(Pmppt(startIndex:end)))*100;

fprintf('\n====================================\n');
fprintf('FINAL RESULTS\n');
fprintf('====================================\n');

fprintf('Output Voltage RMS : %.2f V\n',Vout_rms);
fprintf('Output Current RMS : %.2f A\n',Iout_rms);
fprintf('Output Power       : %.2f W\n',Pout_avg);
fprintf('Efficiency         : %.2f %%\n',Efficiency);

%% ============================================================
% 13. GRAPH 1 - PV POWER
% ============================================================

figure('Color','w','Position',[100 100 1000 550]);

plot(t,Pmppt,'LineWidth',2);

grid on;
box on;

xlabel('Time (s)','FontSize',12);
ylabel('PV Power (W)','FontSize',12);

title('PV Array Power','FontSize',14);

xlim([0 Tsim]);

set(gca,'FontSize',11);

exportgraphics(gcf,'01_PV_Power.png','Resolution',300);

%% ============================================================
% 14. GRAPH 2 - REFERENCE VS OUTPUT VOLTAGE
% ============================================================

figure('Color','w','Position',[100 100 1000 550]);

plot(t,Vref,'LineWidth',1.8);
hold on;

plot(t,Vout,'LineWidth',1.8);

grid on;
box on;

xlabel('Time (s)','FontSize',12);
ylabel('Voltage (V)','FontSize',12);

title('PI Controller Voltage Tracking','FontSize',14);

legend('Reference Voltage','Output Voltage',...
    'Location','best');

xlim([0 Tsim]);

set(gca,'FontSize',11);

exportgraphics(gcf,'02_PI_Voltage_Tracking.png',...
    'Resolution',300);

%% ============================================================
% 15. GRAPH 3 - OUTPUT VOLTAGE ZOOM
% ============================================================

figure('Color','w','Position',[100 100 1000 550]);

zoomIndex = t >= 0.16 & t <= 0.20;

plot(t(zoomIndex),Vref(zoomIndex),'LineWidth',2);
hold on;

plot(t(zoomIndex),Vout(zoomIndex),'LineWidth',2);

grid on;
box on;

xlabel('Time (s)','FontSize',12);
ylabel('Voltage (V)','FontSize',12);

title('Steady-State AC Output Voltage','FontSize',14);

legend('Reference','Filtered Output',...
    'Location','best');

set(gca,'FontSize',11);

exportgraphics(gcf,'03_AC_Output.png','Resolution',300);

%% ============================================================
% 16. GRAPH 4 - OUTPUT CURRENT
% ============================================================

figure('Color','w','Position',[100 100 1000 550]);

plot(t,Iout,'LineWidth',1.8);

grid on;
box on;

xlabel('Time (s)','FontSize',12);
ylabel('Current (A)','FontSize',12);

title('Inverter Output Current','FontSize',14);

xlim([0 Tsim]);

set(gca,'FontSize',11);

exportgraphics(gcf,'04_Output_Current.png',...
    'Resolution',300);

%% ============================================================
% 17. GRAPH 5 - SPWM
% ============================================================

figure('Color','w','Position',[100 100 1100 600]);

% Show only a small interval so pulses are visible
spwmIndex = t >= 0.020 & t <= 0.022;

plot(t(spwmIndex),carrier(spwmIndex),...
    'LineWidth',1.2);

hold on;

plot(t(spwmIndex),modulating(spwmIndex),...
    'LineWidth',2);

stairs(t(spwmIndex),...
    double(PWM(spwmIndex)),...
    'LineWidth',1.2);

grid on;
box on;

xlabel('Time (s)','FontSize',12);
ylabel('Amplitude','FontSize',12);

title('Sinusoidal Pulse Width Modulation (SPWM)',...
    'FontSize',14);

legend('Carrier','Reference Modulating Signal',...
    'PWM Pulse','Location','best');

set(gca,'FontSize',11);

exportgraphics(gcf,'05_SPWM.png','Resolution',300);

%% ============================================================
% 18. GRAPH 6 - OUTPUT POWER
% ============================================================

figure('Color','w','Position',[100 100 1000 550]);

plot(t,Pout,'LineWidth',2);

grid on;
box on;

xlabel('Time (s)','FontSize',12);
ylabel('Output Power (W)','FontSize',12);

title('PV Inverter Output Power','FontSize',14);

xlim([0 Tsim]);

set(gca,'FontSize',11);

exportgraphics(gcf,'06_Output_Power.png',...
    'Resolution',300);

%% ============================================================
% 19. GRAPH 7 - PI CONTROL ERROR
% ============================================================

figure('Color','w','Position',[100 100 1000 550]);

plot(t,error,'LineWidth',1.8);

grid on;
box on;

xlabel('Time (s)','FontSize',12);
ylabel('Voltage Error (V)','FontSize',12);

title('PI Controller Tracking Error','FontSize',14);

xlim([0 Tsim]);

set(gca,'FontSize',11);

exportgraphics(gcf,'07_PI_Error.png',...
    'Resolution',300);

%% ============================================================
% 20. SUMMARY
% ============================================================

fprintf('\n============================================\n');
fprintf(' PSO OPTIMIZED PV INVERTER SUMMARY\n');
fprintf('============================================\n');

fprintf('PV Power           : %.2f W\n',Ppv);
fprintf('DC Link Voltage    : %.2f V\n',Vdc);
fprintf('Reference AC RMS   : %.2f V\n',Vrms_ref);
fprintf('Measured AC RMS    : %.2f V\n',Vout_rms);
fprintf('Output RMS Current : %.2f A\n',Iout_rms);
fprintf('Output Power       : %.2f W\n',Pout_avg);
fprintf('Efficiency         : %.2f %%\n',Efficiency);
fprintf('PSO Kp             : %.6f\n',Kp_opt);
fprintf('PSO Ki             : %.6f\n',Ki_opt);

fprintf('============================================\n');

fprintf('\nGraphs saved as 300 DPI PNG files.\n');
exportgraphics(gcf,'PV_Inverter_Output.png','Resolution',300);


%% ============================================================
% OBJECTIVE FUNCTION FOR PSO
% ============================================================

function cost = inverterObjective(...
    Kp,Ki,Ts,N,Vdc,Vref,Lf,Rf,Cf,Rload)

    % States
    iL = 0;
    Vc = 0;

    integral = 0;

    ISE = 0;
    controlEnergy = 0;

    integral_limit = 300;

    % Ignore first part when calculating cost
    startIndex = round(0.02/Ts);

    for k = 2:N

        % Error
        e = Vref(k)-Vc;

        % Integral
        integral = integral + e*Ts;

        % Anti-windup
        integral = ...
            max(min(integral,integral_limit),...
            -integral_limit);

        % PI
        u = Kp*e + Ki*integral;

        % Modulation
        m = u/Vdc;

        % Saturation
        m = max(min(m,0.95),-0.95);

        % Inverter
        Vinv = m*Vdc;

        % LC filter
        diL = ...
            (Vinv-Rf*iL-Vc)/Lf;

        dVc = ...
            (iL-Vc/Rload)/Cf;

        % Integrate
        iL = iL + diL*Ts;
        Vc = Vc + dVc*Ts;

        % Cost
        if k >= startIndex

            ISE = ISE + e^2*Ts;

            controlEnergy = ...
                controlEnergy + m^2*Ts;

        end

        % Penalize unstable/excessive voltage
        if abs(Vc) > 1000

            cost = 1e12;
            return;

        end

    end

    % Total cost
    cost = ISE + ...
        0.001*controlEnergy;

end