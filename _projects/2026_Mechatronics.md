---
layout: project
title: Mechatronics Robot Compeition 
description: Mechatronics Project 
technologies: [SOLIDWORKS, 3D-Printing, Laser Cutting, Circuits] 
image: /assets/images/Mech_1.JPEG
show_header_image: false 
---

# Overview 
<hr class="section-divider">
The Cube Craze Robot Project challenged teams to design, build, and compete with an autonomous robot capable of collecting and moving cubes within a competition arena. Each match lasted one minute, with the objective of gathering more cubes than the opposing robot by the end of the round.

Working in a team of three, we developed our robot using a provided kit of stock components, supplemented by custom-fabricated and budget-limited purchased parts. The project required balancing mechanical design, electronics integration, sensor feedback, and embedded control logic while staying within competition rules and robot constraints.

The overall goal was to create a reliable, competitive robot that could navigate the arena, detect key field conditions, manipulate cubes effectively, and perform consistently during head-to-head matches.

# Design Overview
<hr class="section-divider">

Our robot’s design strategy prioritized **reliability, board retention, and cube control** over complex navigation. After discussing the competition with students from the previous year, we found that many matches were decided less by advanced strategy and more by whether a robot could remain on the board for the full one-minute match. Based on this, we focused on building a robust system that could consistently detect boundaries, avoid leaving the arena, and continue collecting cubes throughout the round.

Mechanically, the robot used higher-grip wheels to improve traction and maneuverability, along with a custom acrylic housing designed in CAD to capture and retain cubes during motion. The housing was intended to passively collect cubes while reducing the chance of losing them after contact. This approach allowed the robot to accumulate cubes while also remaining durable enough to withstand impacts with other robots.

On the controls side, the original design considered using additional sensors to locate cubes more precisely. However, given the competition’s software constraints and the limited computational complexity allowed on the Arduino, we found that cube-targeting logic was not reliable enough to justify the added complexity. Instead, we focused on refining the QTI border sensors and color sensor to produce consistent, repeatable behavior.

The final control strategy used border detection as the primary reset condition for the robot’s algorithm. When the robot detected the black arena boundary, it would stop, reposition, and restart its search pattern. This created a simple but robust autonomous routine that allowed the robot to stay active for the full match while continuously attempting to collect and retain cubes.

# Competition Analysis
<hr class="section-divider">

Overall, the robot performed very well during the competition. During the round-robin stage, our team went undefeated, which we attribute primarily to the robot’s strong cube enclosure, reliable chassis behavior, and consistent QTI border-detection logic. The robot was able to remain on the board, retain collected cubes, and execute its autonomous routine repeatedly without major mechanical or software failures.

In the final bracket, we won our first elimination match before being eliminated in the round of eight by the team that ultimately finished second overall. The robot functioned as intended throughout the competition, but the later rounds exposed the main limitation of our design: while the robot was reliable, it lacked an active cube acquisition mechanism or a more intelligent search algorithm for locating and collecting cubes.

Our simplified design philosophy became both the robot’s greatest strength and its main weakness. The emphasis on consistency allowed us to perform well across nearly every match, especially against robots that were more failure-prone. However, in a single-elimination bracket, the design was not aggressive enough to consistently outscore teams with more effective cube-gathering strategies.

Despite this limitation, the robot validated our core design approach. By prioritizing reliability, cube retention, and boundary detection, we produced a competitive robot that advanced deep into the tournament and was capable of contending with some of the strongest teams in the class.

# CAD Assembly 
<hr class="section-divider">

<div class="image-row">
  <img src="{{ '/assets/images/Mech_Full_Assem.png'  | relative_url }}" class="torque-image">

</div>

# BOM 
<hr class="section-divider">


The robot was fabricated using a combination of purchased components, laser-cut acrylic parts, and 3D-printed PLA parts. The final estimated cost of the robot was **$37.66**.

The bill of materials for the final design is shown below:

| Part | Fabrication Type | Bounding Box Perimeter | Mass | Unit Cost | Quantity | Total |
|------|------------------|-------------------------|------|-----------|----------|-------|
| Time of Flight Sensor | Purchased | — | — | $14.95 | 1 | $14.95 |
| Wheels | Purchased | — | — | $2.00 | 2 | $4.00 |
| Acrylic Sheet | Stock Material | — | — | $5.00 | 1 | $5.00 |
| Back Plate | Laser Cut | 21.000 in | — | $1.55 | 1 | $1.55 |
| Back Plate Mount | Laser Cut | 8.260 in | — | $0.91 | 1 | $0.91 |
| Side Plate Mount | Laser Cut | 5.880 in | — | $0.79 | 2 | $1.59 |
| Side Plate | Laser Cut | 21.000 in | — | $1.55 | 2 | $3.10 |
| Gate | Laser Cut | 17.500 in | — | $1.38 | 1 | $1.38 |
| Color Sensor Mount | Laser Cut | 7.720 in | — | $0.89 | 1 | $0.89 |
| Gate Stop | Laser Cut | 17.000 in | — | $1.35 | 1 | $1.35 |
| Gate Mount | 3D Printed PLA | — | 1.180 g | $1.47 | 2 | $2.94 |
| **Total** |  |  |  |  |  | **$37.66** |
{: .material-table}

# Robot Image Gallery 
<hr class="section-divider">

<div class="image-row">
  <img src="{{ '/assets/images/Mech_1.JPEG'  | relative_url }}" class="torque-image">
  <img src="{{ '/assets/images/Mech_2.JPEG'  | relative_url }}" class="torque-image">
  <img src="{{ '/assets/images/Mech_3.JPEG'  | relative_url }}" class="torque-image">
  <img src="{{ '/assets/images/Mech_4.JPEG'  | relative_url }}" class="torque-image">
  <img src="{{ '/assets/images/Mech_6.JPEG'  | relative_url }}" class="torque-image">
  <img src="{{ '/assets/images/Mech_7.JPEG'  | relative_url }}" class="torque-image">
</div>

# Design Report
<hr class="section-divider">
<p>
  <a href="{{ '/assets/Mech_report.pdf' | relative_url }}" target="_blank">
    View Full PDF 
  </a>
</p>

<iframe src="{{ '/assets/Mech_report.pdf' | relative_url }}" 
        width="100%" 
        height="800px">
</iframe>





