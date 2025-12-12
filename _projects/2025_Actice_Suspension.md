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





