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


q_dot_min_body_frame = [-0.3 -0.3];
q_dot_max_body_frame = [ 0.3  0.3];



Vx_min_b = q_dot_min_body_frame(1);
Vx_max_b = q_dot_max_body_frame(1);

Vy_min_b = q_dot_min_body_frame(2);
Vy_max_b = q_dot_max_body_frame(2);
% 
% [Vx_min_b  Vx_max_b;
%  Vy_min_b  Vy_max_b
%     ]


v1_b = Vx_max_b*i_ + Vy_max_b*j_;
v2_b = Vx_min_b*i_ + Vy_max_b*j_;
v3_b = Vx_min_b*i_ + Vy_min_b*j_;
v4_b = Vx_max_b*i_ + Vy_min_b*j_;



phi_deg= linspace(0, 360, 73);
%phi = deg2rad(180);

for i=1:length(phi_deg)
    r = cos(deg2rad(phi_deg(i))/2) + k_*sin(deg2rad(phi_deg(i))/2);
    v1_a = vec3(Ad(r, v1_b));
    v2_a = vec3(Ad(r, v2_b));
    v3_a = vec3(Ad(r, v3_b));
    v4_a = vec3(Ad(r, v4_b));
    vx_w = [v1_a(1), v2_a(1), v3_a(1), v4_a(1)];
    vy_w = [v1_a(2), v2_a(2), v3_a(2), v4_a(2)];
    Vx_min_w = min(vx_w);
    Vx_max_w = max(vx_w);
    
    Vy_min_w = min(vy_w);
    Vy_max_w = max(vy_w);

    q_dot_min_inertial_frame(:,i) = [Vx_min_w; Vy_min_w];
    q_dot_max_inertial_frame(:,i) = [Vx_max_w; Vy_max_w];

end



subplot(2,1,1)

plot(phi_deg, q_dot_min_inertial_frame(1, :));
hold on
plot(phi_deg, q_dot_max_inertial_frame(1, :));
hold on
plot(phi_deg, q_dot_min_body_frame(1)*ones(length(q_dot_min_inertial_frame(1, :))))
hold on
plot(phi_deg, q_dot_max_body_frame(1)*ones(length(q_dot_min_inertial_frame(1, :))))

subplot(2,1,2)
plot(phi_deg, q_dot_min_inertial_frame(2, :));
hold on
plot(phi_deg, q_dot_max_inertial_frame(2, :));
hold on
plot(phi_deg, q_dot_min_body_frame(2)*ones(length(q_dot_min_inertial_frame(1, :))))
hold on
plot(phi_deg, q_dot_max_body_frame(2)*ones(length(q_dot_min_inertial_frame(1, :))))




% phi_dot_max = 1;
% 
% 
Vx_min_b = -1;
Vx_max_b = 1;

Vy_min_b = -1;
Vy_max_b = 1;




v1_b = Vx_max_b*i_ + Vy_max_b*j_;
v2_b = Vx_min_b*i_ + Vy_max_b*j_;
v3_b = Vx_min_b*i_ + Vy_min_b*j_;
v4_b = Vx_max_b*i_ + Vy_min_b*j_;
phi = 45;
r = cos(deg2rad(phi)/2) + k_*sin(deg2rad(phi)/2);
v1_a = vec3(Ad(r, v1_b))
v2_a = vec3(Ad(r, v2_b))
v3_a = vec3(Ad(r, v3_b))
v4_a = vec3(Ad(r, v4_b))

vx_w = [v1_a(1), v2_a(1), v3_a(1), v4_a(1)]
vy_w = [v1_a(2), v2_a(2), v3_a(2), v4_a(2)]
Vx_min_w = min(vx_w);
Vx_max_w = max(vx_w);

Vy_min_w = min(vy_w);
Vy_max_w = max(vy_w);


[Vx_min_w  Vx_max_w;
 Vy_min_w  Vy_max_w
    ]