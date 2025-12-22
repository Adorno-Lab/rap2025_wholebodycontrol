function [q_dot_min_body_frame, q_dot_max_body_frame] = convert_to_body_frame(q_dot_min_inertial_frame ,q_dot_max_inertial_frame , r)


    include_namespace_dq;
    
    
    
    Vx_min_w = q_dot_min_inertial_frame(1);
    Vx_max_w = q_dot_max_inertial_frame(1);
    
    Vy_min_w = q_dot_min_inertial_frame(2);
    Vy_max_w = q_dot_max_inertial_frame(2);

    
    
    v1_w = Vx_max_w*i_ + Vy_max_w*j_;
    v2_w = Vx_min_w*i_ + Vy_max_w*j_;
    v3_w = Vx_min_w*i_ + Vy_min_w*j_;
    v4_w = Vx_max_w*i_ + Vy_min_w*j_;
    
    v1_a = vec3(Ad(r, v1_w));
    v2_a = vec3(Ad(r, v2_w));
    v3_a = vec3(Ad(r, v3_w));
    v4_a = vec3(Ad(r, v4_w));

    vx_w = [v1_a(1), v2_a(1), v3_a(1), v4_a(1)];
    vy_w = [v1_a(2), v2_a(2), v3_a(2), v4_a(2)];
    Vx_min_w = min(vx_w);
    Vx_max_w = max(vx_w);
    
    Vy_min_w = min(vy_w);
    Vy_max_w = max(vy_w);
    
    q_dot_min_body_frame = [Vx_min_w; Vy_min_w];
    q_dot_max_body_frame = [Vx_max_w; Vy_max_w];
end