
%% Step 1: Define Transfer Function & Open Loop Analysis
% Clearing workspace, command window and figures to 
% ensure clean execution environment
clc; clear; close all;

% The drone's vertical dynamics are modelled using a
% second order transfer function G(s) = 1/(s^2 + 2s + 5)
% where:
%   - Numerator = 1 (unit gain input)
%   - Denominator = s^2 + 2s + 5 (natural drone dynamics)
%   - Input = thrust command
%   - Output = altitude in metres
num = [1];
den = [1 2 5];
G = tf(num, den);

disp('=== Plant Transfer Function ===');
G

%% Pole Analysis — Checking Open Loop Stability
% Poles are values of s that make denominator = 0
% Negative real part = stable, Positive real part = unstable
p = pole(G);
disp('=== Poles of G(s) ===');
for i = 1:length(p)
    fprintf('Pole %d = %.2f + %.2fi\n', i, real(p(i)), imag(p(i)));
    if real(p(i)) < 0
        fprintf('  Real part = %.2f → NEGATIVE → STABLE ✓\n', real(p(i)));
    else
        fprintf('  Real part = %.2f → POSITIVE → UNSTABLE ✗\n', real(p(i)));
    end
end

% Overall stability conclusion
if all(real(p) < 0)
    disp('=== RESULT: Open Loop System is STABLE ✓ ===');
else
    disp('=== RESULT: Open Loop System is UNSTABLE ✗ ===');
end

%% Graph 1: Open Loop Step Response
% Shows drone behaviour WITHOUT any controller
% Expected: drone only reaches 0.2m due to DC gain = 1/5
% This confirms the need for a controller
figure(1);
step(G);
title('Graph 1: Open-Loop Step Response (No Controller)');
xlabel('Time (s)');
ylabel('Altitude (m)');
grid on;
text(0.5, 0.15, 'No controller — drone does not reach target', ...
    'Color', 'red', 'FontSize', 10);

%% Graph 2: Pole Zero Map
% X marks show pole locations on complex plane
% Left side = stable region, Right side = unstable region
% Both poles at s = -1 ± 2j confirm open loop stability
figure(2);
pzmap(G);
title('Graph 2: Pole-Zero Map — X marks show poles');
grid on;
xline(0, 'r--', 'LineWidth', 1.5);
text(-0.5, 2.5, 'STABLE SIDE', 'Color', 'green', ...
    'FontSize', 11, 'FontWeight', 'bold');
text(0.1, 2.5, 'UNSTABLE SIDE', 'Color', 'red', ...
    'FontSize', 11, 'FontWeight', 'bold');

%% Graph 3: Bode Plot
% Shows frequency response of open loop system
% Used to analyse gain and phase margins
% which confirm closed loop stability and robustness
figure(3);
bode(G);
title('Graph 3: Bode Plot — Frequency Response of Drone');
grid on;

%% Step 2: Auto Tuned PID Controller Design
% pidtune() uses loop shaping and frequency domain analysis
% to automatically calculate optimal Kp, Ki, Kd values
% that satisfy stability and performance requirements:
%   - Overshoot < 10%
%   - Settling time < 3 seconds
%   - Steady state error = 0
[C_auto, info_auto] = pidtune(G, 'PID');
disp('=== Auto-Tuned PID Controller ===');
C_auto

% Closing the feedback loop — connecting controller output
% back to input so drone continuously corrects its altitude
% Formula: T = (C*G) / (1 + C*G)
T_auto = feedback(C_auto * G, 1);

%% Graph 4: Closed Loop Step Response
% Shows drone behaviour WITH PID controller
% Expected: reaches 1m with overshoot < 10%
% and settling time < 3 seconds
figure(4);
step(T_auto);
title('Graph 4: Closed-Loop Step Response - Auto Tuned PID');
xlabel('Time (s)');
ylabel('Altitude (m)');
grid on;

% Extracting performance metrics automatically
% to verify all design specifications are met
info2 = stepinfo(T_auto);
fprintf('\n=== Auto-Tuned Performance Metrics ===\n');
fprintf('Overshoot     : %.2f%%\n', info2.Overshoot);
fprintf('Settling Time : %.2f s\n', info2.SettlingTime);
fprintf('Rise Time     : %.2f s\n', info2.RiseTime);

% Automatically checking if requirements are satisfied
if info2.Overshoot < 10
    disp('Overshoot is within limit ✓');
else
    disp('Overshoot exceeds limit ✗ — increase Kd');
end

if info2.SettlingTime < 3
    disp('Settling Time is within limit ✓');
else
    disp('Settling Time exceeds limit ✗ — adjust Ki');
end

%% Interactive PID Tuner
% Opens visual tuning interface with sliders
% Slider 1 (Response Time) — controls how fast drone reaches target
% Slider 2 (Transient Behaviour) — controls how much drone wobbles
% Move sliders until response meets specifications
% Then click Export to save new Kp Ki Kd values
pidTuner(G, 'PID')

%% Step 3: Disturbance Simulation at t = 5 seconds
% Simulating real world wind disturbance to test robustness
% t = timeline from 0 to 20 seconds
t = 0:0.01:20;

% r = reference signal — target altitude = 1 metre always
r = ones(size(t));

% d = disturbance signal — wind hits at exactly t = 5 seconds
% with magnitude 10, simulating a sudden wind gust
d = zeros(size(t));
d(t >= 5 & t < 5.01) = 10;

% S = sensitivity function = 1 - T_auto
% Describes how the system reacts to external disturbances
% A smaller S means disturbances have less effect — better robustness
S = 1 - T_auto;

% Simulating drone response to reference command
y_ref = lsim(T_auto, r, t);

% Simulating drone response to wind disturbance
y_dist = lsim(S, d, t);

% Combined response = reference tracking + disturbance effect
y_total = y_ref + y_dist;

%% Graph 5: Disturbance Response
% Shows drone altitude before and after wind hits at t=5s
% Controller should bring drone back to 1m quickly
figure(5);
plot(t, r, 'k--', 'LineWidth', 1.5); hold on;
plot(t, y_total, 'b', 'LineWidth', 2);
xline(5, 'r--', 'Disturbance at t=5s');
title('Graph 5: Closed-Loop Response with Disturbance at t=5s');
xlabel('Time (s)');
ylabel('Altitude (m)');
legend('Target Altitude', 'Actual Altitude');
grid on;

%% Step 4: Stability Margins Analysis
% L = open loop transfer function (controller x drone)
% Used to calculate gain and phase margins
L = C_auto * G;

%% Graph 6: Gain and Phase Margin
% Gain margin — how much gain can increase before instability
% Good value = more than 6 dB
% Phase margin — how much delay system can handle
% Good value = between 30 and 60 degrees
figure(6);
margin(L);
title('Graph 6: Gain and Phase Margin — Stability Analysis');
grid on;

% Extracting exact margin values
[Gm, Pm, Wcg, Wcp] = margin(L);
GmdB = 20*log10(Gm);

fprintf('\n=== Stability Margins ===\n');
fprintf('Gain Margin  : %.2f dB\n', GmdB);
fprintf('Phase Margin : %.2f degrees\n', Pm);

if GmdB > 6
    disp('Gain Margin is GOOD ✓');
else
    disp('Gain Margin is LOW ✗');
end

if Pm > 30
    disp('Phase Margin is GOOD — system is robust ✓');
else
    disp('Phase Margin is LOW — system may be unstable ✗');
end


%% Step 5: Final Comparison Plot
% Comparing open loop vs closed loop performance
% Visually proves the PID controller improved the system
figure(8);
hold on;
step(G, 'r--');
step(T_auto, 'g');
legend('Open Loop - No Controller', 'Auto Tuned PID Controller');
title('Graph 8: Final Comparison — Open Loop vs PID Controller');
xlabel('Time (s)');
ylabel('Altitude (m)');
grid on;
