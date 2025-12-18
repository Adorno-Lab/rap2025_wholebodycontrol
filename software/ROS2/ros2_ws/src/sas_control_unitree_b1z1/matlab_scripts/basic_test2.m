clear
clc
close all
clear all

i_ = DQ.i;
j_ = DQ.j;
k_ = DQ.k;
E_ = DQ.E;
C4 = DQ.C4;
C8 = DQ.C8;


q_dot_min_body_frame = [-1 -1 -1];
q_dot_max_body_frame = [ 1  1  1];

Vx_min_b = q_dot_min_body_frame(1);
Vx_max_b = q_dot_max_body_frame(1);

Vy_min_b = q_dot_min_body_frame(2);
Vy_max_b = q_dot_max_body_frame(2);

Vw_min_b = q_dot_min_body_frame(3);
Vw_max_b = q_dot_max_body_frame(3);



phi_deg = 45;
c = cos(deg2rad(phi_deg));
s = sin(deg2rad(phi_deg));

Vx_min_w = Vx_min_b*c - Vy_max_b*s;

Vx_max_w = Vx_max_b*c - Vy_min_b*s;

Vy_min_w = Vx_min_b*s + Vy_min_b*c;
Vy_max_w = Vx_max_b*s + Vy_max_b*c;

[Vx_min_w  Vx_max_w;
 Vy_min_w  Vy_max_w
    ]