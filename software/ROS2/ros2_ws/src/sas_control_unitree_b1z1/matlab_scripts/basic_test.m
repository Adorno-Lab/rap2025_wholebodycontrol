clear
clc
close all

using DQ_robotics_namespace;


q_dot_min_body_frame = [-0.3 -0.3 -0.3];
q_dot_max_body_frame = [0.3 0.3 0.3];


v1_b = Vx_max_b*i_ + Vy_max_b*j_;
v2_b = Vx_min_b*i_ + Vy_max_b*j_;
v3_b = Vx_min_b*i_ + Vy_min_b*j_;
v4_b = Vx_max_b*i_ + Vy_min_b*j_;

q_dot_min_inertial_frame = q_dot_min_inertial';
q_dot_max_inertial_frame = q_dot_max_inertial';