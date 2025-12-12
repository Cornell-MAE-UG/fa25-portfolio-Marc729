---
layout: project
title: Active Suspension Model
description: System Dynamics Project
technologies: [MATLAB] 
image: /assets/images/Suspension_model.png
show_header_image: false 
---

# Overview
<hr class="section-divider">

The goal of this project was to design a model of a qaurter-car active suspension. 

# Model 
<hr class="section-divider">

The quarter-car active suspension model includes a sprung mass m<sub>b</sub> (the car body) and an unsprung mass m<sub>w</sub> (the wheel assembly). These masses are connected by a suspension made up of a spring and a damper, and an actuator applies a control force between them. The tire is modeled with a stiffness k<sub>t</sub>. The system outputs are body displacement (x<sub>b</sub>), body acceleration (a), and suspension deflection (d<sub>2</sub>). The inputs are the road disturbance (r) and the actuator force (f) 

![Description]({{ '/assets/images/Suspension_model.png' | relative_url }}){: .torque-image}


# State Space 
<hr class="section-divider">

![Description]({{ '/assets/images/Suspension_statespace.png' | relative_url }}){: .torque-image}

| m<sub>b</sub> (kg) | m<sub>w</sub> (kg) | k<sub>s</sub> (kN/m) | k<sub>t</sub> (kN/m) | b<sub>s</sub> (kNs/m) |
|--------------------|--------------------|----------------------|----------------------|------------------------|
| 250                | 30                 | 20                   | 150                  | 1.5                    |
{: .material-table }

# Control Method 
<hr class="section-divider">

LQR is a method used to control a system while keeping it stable and close to its desired state, while balancing performance and energy use. It is typically applied to systems written in state-space form and uses the A and B matrices as inputs. It also requires two weighting matrices, Q and R, which set the emphasis on state error and control effort, respectively. A cost function is then minimized to produce the feedback gain matrix K.

In our specific system we assigned weights to the system outputs with a greater emphasis on body displacement and body acceleration over suspension deflection. Our specific K matrix calculation are shown here: 

$$
Q_y =
\begin{bmatrix}
1e6 & 0   & 0 \\
0   & 1e2 & 0 \\
0   & 0   & 1e6
\end{bmatrix}
$$

$$
Q = C^T Q_y C = 1e9
\begin{bmatrix}
 6.401 &  0.480 & -6.400 & -0.480 \\
 0.480 &  0.036 & -0.480 & -0.036 \\
-6.400 & -0.480 &  6.400 &  0.480 \\
-0.480 & -0.036 &  0.480 &  0.036
\end{bmatrix}
$$

For our R matrix we used R = [0.5] 

# Resonant Frequencies 
<hr class="section-divider">

To study the system response, we decided to first find the resonant frequencies of the road disturbance to the body displacement, body acceleration, and suspension deflection. Studying the system at these frequencies will provide the worst case scenarios for the model. 

![Description]({{ '/assets/images/Suspension_bode.png' | relative_url }}){: .torque-image}

The results showed that the resonant frequencies for the highest peak of body displacement and shock deflection occur at 8.59 [rad/s] and at 73.66 [rad/s] for body acceleration.  

# Sinusoidal Road Disturbance Response 
<hr class="section-divider">

Our first simulation was with a sinusoidal road disturbance of amplitude 10 [cm] and frequency 8.59 [rad/s]. The equation used for this disturbance was r(t)=0.1sin(8.59t). 

<div class="image-row">
  <img src="{{ '/assets/images/Suspension_results1.png' | relative_url }}" class="torque-image">
  <img src="{{ '/assets/images/SUspension_results12.png' | relative_url }}" class="torque-image">
  <img src="{{ '/assets/images/Suspension_results13.png' | relative_url }}" class="torque-image">
</div>

The simulation results show a substantial reduction in body travel, suspension deflection, and body acceleration for the closed-loop system compared to the passive suspension. The LQR model was especially effective in reducing suspension, deflection, and body acceleration, achieving steady-state amplitude ratios of 0.11 for both. The steady-state amplitude ratio for body travel was 0.52. The actuator force reached a peak of 3.4 kN and settled into a steady-state amplitude of about 1.9 kN. 

<div class="mini-break"></div>  

Our second simulation was with a sinusoidal road disturbance of amplitude 10 [cm] and frequency 73.66 [rad/s]. The equation used for this disturbance was r(t)=0.1sin(73.66t). 

<div class="image-row">
  <img src="{{ '/assets/images/Suspension_results2.png' | relative_url }}" class="torque-image">
  <img src="{{ '/assets/images/Suspension_results22.png' | relative_url }}" class="torque-image">
  <img src="{{ '/assets/images/Suspension_results23.png' | relative_url }}" class="torque-image">
</div>

The simulation results likewise indicate a notable decrease in body travel, suspension deflection, and body acceleration for the closed-loop system relative to the passive suspension. In this case, the LQR controller again proved highly effective at limiting suspension deflection and body acceleration, achieving steady-state amplitude ratios of 0.17 for both. The steady-state amplitude ratio for body travel was 0.97. The actuator force reached a peak of 14.8 kN and settled to a steady-state amplitude of about 12.2 kN. Although the amplitude ratio for body travel is close to 1, it is worth noting that the absolute displacement is relatively small—especially compared to the magnitudes of deflection and acceleration.

# Step Road Disturbance Response 
<hr class="section-divider">

Our third simulation was with a step road disturbance of amplitude 10 [cm]. The equation used for this disturbance was r(t)=0.1ones(size(t)). 

<div class="image-row">
  <img src="{{ '/assets/images/Suspension_results3.png' | relative_url }}" class="torque-image">
  <img src="{{ '/assets/images/SUspension_results32.png' | relative_url }}" class="torque-image">
</div>

The simulation results show a substantial reduction in peak body travel, suspension deflection, and body acceleration for the closed-loop system compared to the passive suspension. The LQR controller was particularly effective in lowering the peak suspension deflection and peak body acceleration: the peak deflection dropped from 0.11 m to 0.04 m, and the peak acceleration decreased from 28.2 m/s² to 8.7 m/s². Peak body travel also saw a slight reduction, from 0.15 m to 0.14 m.

# Discussion 
<hr class="section-divider">

Overall, across all three test cases, the LQR model performed better than the passive model. It consistently showed a large improvement in reducing body acceleration and suspension displacement, while the improvement in body travel was smaller. The body displacement could be reduced further by adjusting the Q matrix; however, doing so would increase suspension deflection and body acceleration, which are more critical for ride comfort and physical limits such as shock travel. The results could also be improved by increasing the value of the R matrix, though this would lead to higher actuator forces. Additionally, simulating with different amplitudes or frequencies for the road would lead to different results, but these results were determined to be a worst case scenario due to the road oscillating at the system's natural frequency.

# MATLAB Code  
<hr class="section-divider">
**MATLAB Script:** 

```matlab
clear; clc; close all;

%% Parameters 
mb = 250; mw = 30;
ks = 20000; bs = 1500;
kt = 150000;

%% State-space model

% x = [xb xb. xw xw.] | u = [r f] | y = [xb ds ab] 

% A Matric 
A = [    0       1          0           0 
      -ks/mb  -bs/mb      ks/mb       bs/mb 
         0        0         0           1 
       ks/mw   bs/mw   (-ks-kt)/mw   -bs/mw ];

% B Matrix 
B = [ 0         0 
      0        1/mb 
      0         0 
      kt/mw   -1/mw ];

%  C Matrix 
C =  [    1        0       0       0
          1        0      -1       0 
       -ks/mb   -bs/mb   ks/mb   bs/mb ];

% D Matrix 
D = [ 0    0 
      0    0 
      0   1/mb ];

% Create State Space 
sys_sus = ss(A,B,C,D); 

sys_sus.InputName = {'r', 'F'}; 
sys_sus.OutputName = {'xb', 'ds', 'ab'}; 
sys_sus.StateName = {'Body Travel (m)', 'Body Velocity (m/s)', ...  
                   'Wheel travel (m)', 'Wheel Velovity (m/s)'}; 

sys_tf = tf(sys_sus); 
sys_zpk = zpk(sys_sus); 

%% Plot Bodeplot from road input
figure 
color = ['r', 'b', 'g']; 
for i = 1:size(sys_sus, 1)
   bodemag(sys_sus(i,1), color(i)); 
   hold on 
end 

legend(sys_sus.OutputName)
title('System Reponse from r')

for i = 1:size(sys_sus, 1) 
    poles = sys_zpk.P{i,1}; 
    wn = unique(abs(poles)); 
    resp = abs(freqresp(sys_sus(i,1), wn)); 
    [peakMag, inx] = max(resp);
    peakFreq = wn(inx); 

    semilogx(peakFreq, mag2db(peakMag), '*', LineWidth=2, HandleVisibility='off', Color=color(i)); 
    hold on 
    semilogx([peakFreq, peakFreq], [-200, mag2db(peakMag)], ':', LineWidth=2, HandleVisibility='off', Color=color(i))
    hold on 
end 

%% Split disturbance input and control input
Bd = B(:,1);   % road disturbance input
Bc = B(:,2);   % control input (active force F)
Dd = D(:,1);   % direct feedthrough from r
Dc = D(:,2);   % direct feedthrough from F


%% LQR design (output weighting)
Wy = diag([1e6, 1e2, 1e6]);    % weights on [xb; ds; ab]
Q  = C' * Wy * C;              % project into state space
R  = 0.5;

[K, S, e] = lqr(A, Bc, Q, R); 

%% Closed-loop system from road r -> outputs [xb; ds; ab]
Acl = A - Bc*K;   % closed-loop A matrix

Bcl = Bd;         % only road input in this model
Ccl = C;
Dcl = Dd;

sys_cl = ss(Acl, Bcl, Ccl, Dcl);   % <--- THIS defines sys_cl
sys_cl.InputName  = {'r'};
sys_cl.OutputName = {'xb', 'ds', 'ab'};

%% Passive system (no control, F = 0)
sys_passive = ss(A, Bd, C, Dd);
sys_passive.InputName  = {'r'};
sys_passive.OutputName = {'xb', 'ds', 'ab'};

%% Road input:
t = 0:0.001:5;
r = 0.1*ones(size(t)); 

[y_pass, t_pass] = lsim(sys_passive, r, t);   % passive response
[y_act,  t_act ] = lsim(sys_cl,      r, t);   % active LQR response

%% Plot outputs
figure;
subplot(3,1,1)
plot(t_pass, y_pass(:,1), 'LineWidth', 1.5); hold on;
plot(t_act,  y_act(:,1),  '--', 'LineWidth', 1.5);
grid on;
ylabel('x_b (m)');
legend('Passive','LQR Active');
title('Body Travel');

subplot(3,1,2)
plot(t_pass, y_pass(:,2), 'LineWidth', 1.5); hold on;
plot(t_act,  y_act(:,2),  '--', 'LineWidth', 1.5);
grid on;
ylabel('d_s (m)');
legend('Passive','LQR Active');
title('Suspension Deflection');

subplot(3,1,3)
plot(t_pass, y_pass(:,3), 'LineWidth', 1.5); hold on;
plot(t_act,  y_act(:,3),  '--', 'LineWidth', 1.5);
grid on;
ylabel('a_b (m/s^2)');
xlabel('Time (s)');
legend('Passive','LQR Active');
title('Body Acceleration');

%% Plot control force
% Get state response for closed-loop
x_act = lsim(ss(Acl, Bcl, eye(4), zeros(4,1)), r, t);
F_act = - (K * x_act.').';   % F = -Kx

figure;
plot(t, F_act, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('F (N)');
title('Active Suspension Force (LQR)');

%% Steady-state amplitude ratios (LQR / Passive) for sinusoidal input

% sinusoid used above: r = 0.1*sin(8*t);
omega = wn(1);                  % rad/s
T = 2*pi/omega;             % period of the sinusoid
nCycles = 5;                % how many final cycles to use for steady state

t_ss_start = t(end) - nCycles*T;   % start time for steady-state window
idx_ss = t >= t_ss_start;          % logical indices for steady state

% steady-state portions of the outputs
y_pass_ss = y_pass(idx_ss, :);     % passive outputs in steady state
y_act_ss  = y_act(idx_ss,  :);     % LQR outputs in steady state

% estimate amplitude as half of peak-to-peak for each output
A_pass = (max(y_pass_ss) - min(y_pass_ss))/2;   % 1x3: [xb, ds, ab]
A_act  = (max(y_act_ss)  - min(y_act_ss))/2;    % 1x3

% amplitude ratios: LQR / Passive
amp_ratio = A_act ./ A_pass;   % 1x3

disp('Steady-state amplitude ratios (LQR / Passive) for [xb, ds, ab]:');
disp(amp_ratio);

% bar plot of the ratios
figure;
bar(amp_ratio);
set(gca, 'XTickLabel', {'x_b','d_s','a_b'});
ylabel('Amplitude ratio (LQR / Passive)');
title('Steady-state amplitude ratios for sinusoidal road input');
grid on;
```
{: .code-scroll}