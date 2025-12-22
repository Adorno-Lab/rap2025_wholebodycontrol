clear
clc
close all
clear all


include_namespace_dq;

q_dot_min_body_frame = [-1 -1];
q_dot_max_body_frame = [ 1  1];



phi_deg= linspace(0, 360, 73);
q_dot_min_inertial_frame = zeros(2, length(phi_deg));
q_dot_max_inertial_frame = zeros(2, length(phi_deg));



for i=1:length(phi_deg)
    r = cos(deg2rad(phi_deg(i))/2) + k_*sin(deg2rad(phi_deg(i))/2);
    [q_dot_min_inertial_frame(:,i), q_dot_max_inertial_frame(:,i)] = convert_to_world_frame(q_dot_min_body_frame, q_dot_max_body_frame, r);
end


q_dot_min_body_frame_computed = zeros(2, length(phi_deg));
q_dot_max_body_frame_computed = zeros(2, length(phi_deg));

for i=1:length(phi_deg)
   [q_dot_min_body_frame_computed(:,i), q_dot_max_body_frame_computed(:,i)] = convert_to_body_frame(q_dot_min_inertial_frame(:,i), q_dot_max_inertial_frame(:,i), conj(r));
end

q_dot_min_inertial_frame_new = zeros(2, length(q_dot_min_inertial_frame));
q_dot_max_inertial_frame_new = zeros(2, length(q_dot_max_inertial_frame));


for i=1:length(q_dot_max_inertial_frame)
    phi = deg2rad(phi_deg(i));

    R = [cos(phi) sin(phi);
        -sin(phi) cos(phi)];
    q_dot_min_inertial_frame_new(:,i) =  R*q_dot_min_body_frame';
    q_dot_max_inertial_frame_new(:,i) =  R*q_dot_max_body_frame';


end

h1 = figure;

subplot(2,1,1)

plot(phi_deg, q_dot_min_inertial_frame(1, :), "--b",'LineWidth', 2);
hold on
plot(phi_deg, q_dot_max_inertial_frame(1, :), "--b",'LineWidth', 2) ;
hold on
plot(phi_deg, q_dot_min_body_frame_computed(1,:), "m", 'LineWidth', 1) 
hold on
plot(phi_deg, q_dot_max_body_frame_computed(1,:), "m", 'LineWidth', 1) 
hold on
plot(phi_deg, q_dot_min_body_frame(1)*ones(length(q_dot_min_inertial_frame(1, :))), "--r", 'LineWidth', 3) 
hold on
plot(phi_deg, q_dot_max_body_frame(1)*ones(length(q_dot_min_inertial_frame(1, :))), "--r", 'LineWidth', 3) 

hold on
plot(phi_deg, q_dot_min_inertial_frame_new(1,:), "--g", 'LineWidth', 1) 
hold on
plot(phi_deg, q_dot_max_inertial_frame_new(1,:), "g", 'LineWidth', 1) 






legend({'lb_{wf}', 'ub_{wf}','lbproj_{bf}', 'ubproj_{bf}','lb_{bf}', 'ub_{bf}'},'Location','northeast')
title(['vx'])
xlabel(['\theta'])


set(gca, 'FontSize', 20);
fig = gcf;
fig.Color = [1 1 1];
box('off');



subplot(2,1,2)
plot(phi_deg, q_dot_min_inertial_frame(2, :),"--b",'LineWidth', 2) ;
hold on
plot(phi_deg, q_dot_max_inertial_frame(2, :),"--b",'LineWidth', 2);
hold on
hold on
plot(phi_deg, q_dot_min_body_frame_computed(2,:), "m", 'LineWidth', 1) 
hold on
plot(phi_deg, q_dot_max_body_frame_computed(2,:), "m", 'LineWidth', 1) 
hold on
plot(phi_deg, q_dot_min_body_frame(2)*ones(length(q_dot_min_inertial_frame(1, :))),"--r", 'LineWidth', 3) 
hold on
plot(phi_deg, q_dot_max_body_frame(2)*ones(length(q_dot_min_inertial_frame(1, :))),"--r", 'LineWidth', 3) 

hold on
plot(phi_deg, q_dot_min_inertial_frame_new(2,:), "--g", 'LineWidth', 1) 
hold on
plot(phi_deg, q_dot_max_inertial_frame_new(2,:), "g", 'LineWidth', 1) 

set(gca, 'FontSize', 20);
fig = gcf;
fig.Color = [1 1 1];
box('off');

legend({'lb_{wf}', 'ub_{wf}','lbproj_{bf}', 'ubproj_{bf}','lb_{bf}', 'ub_{bf}'},'Location','northeast')
title(['vy'])
xlabel(['\theta'])


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