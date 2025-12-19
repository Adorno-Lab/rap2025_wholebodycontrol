function [q_dot_min_inertial_frame ,q_dot_max_inertial_frame] = convert_to_world_frame(q_dot_min_body_frame,q_dot_max_body_frame, r)
%CONVERT_TO_WORLD_FRAME Summary of this function goes here
%   Detailed explanation goes here
    include_namespace_dq;
    
    
    
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
    
    q_dot_min_inertial_frame = [Vx_min_w; Vy_min_w];
    q_dot_max_inertial_frame = [Vx_max_w; Vy_max_w];
end