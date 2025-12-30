

clear all
close all
clc

load("sas_log_2025_12_30_18_27_47.mat")
%load("/home/s55322jq/sas_log_2025_12_04_13_45_13.mat")



u_cmd = u';
ub_cmd = ub'; 

q_dot_min_body_frame = q_dot_min';
q_dot_max_body_frame = q_dot_max';

q_dot_min_inertial_frame = q_dot_min_inertial';
q_dot_max_inertial_frame = q_dot_max_inertial';

robot_pose = x';
configuration = q';

ctwist = ctwist';
nctwist(1,:) = ctwist(2,:);
nctwist(2,:) = ctwist(3,:);
nctwist(3,:) = ctwist(1,:);

h1 = figure;
set(h1, 'DefaultTextFontSize', 10);
set(h1, 'DefaultAxesFontSize', 10); % [pt]
set(h1, 'DefaultAxesFontName', 'mwa_cmr10');
set(h1, 'DefaultTextFontName', 'mwa_cmr10');
set(h1, 'Units', 'centimeters');
pos = get(h1, 'Position');
pos(3) = 2*20; % Select the width of the figure in [cm] 17
pos(4) = 2*10; % Select the height of the figure in [cm] 6
set(h1, 'Position', pos);
set(h1, 'PaperType', 'a4letter');
set(h1,'PaperPositionMode','auto')
set(h1, 'Renderer', 'Painters');
fontsize = 20;


q_dot_min_body_frame_computed = zeros(2, length(q_dot_min_inertial_frame));
q_dot_max_body_frame_computed = zeros(2, length(q_dot_max_inertial_frame));
for i=1:length(q_dot_max_inertial_frame)
    phi = configuration(3,i);
    r = cos(phi/2)+DQ.k*sin(phi/2);
   [q_dot_min_body_frame_computed(:,i), q_dot_max_body_frame_computed(:,i)] = convert_to_body_frame(q_dot_min_inertial_frame(:,i), q_dot_max_inertial_frame(:,i), conj(r));
end


q_dot_min_inertial_frame_new = zeros(2, length(q_dot_min_inertial_frame));
q_dot_max_inertial_frame_new = zeros(2, length(q_dot_max_inertial_frame));


for i=1:length(q_dot_max_inertial_frame)
    phi = configuration(3,i);
    r = cos(phi/2)+DQ.k*sin(phi/2);

    R = [cos(phi) sin(phi);
        -sin(phi) cos(phi)];
    q_dot_min_inertial_frame_new(:,i) =  R*q_dot_min_body_frame(1:2,i);
    q_dot_max_inertial_frame_new(:,i) =  R*q_dot_max_body_frame(1:2,i);


end



for i=1:3

    subplot(3,1,i)
    plot(u_cmd(i,:),"--b",'LineWidth', 2)  
    hold on
    plot(q_dot_min_body_frame(i,:), "--r", 'LineWidth', 2) 
    hold on
    plot(q_dot_max_body_frame(i,:), "--r", 'LineWidth', 2)


    if i <= 3
            hold on
            plot(ub_cmd(i,:),"r", 'LineWidth', 2) 
            hold on 
    end
    title(['u_',num2str(i)])



    % plot(nctwist(i,:),":k", 'LineWidth', 2) 
    % hold on
    
    % 
    % 
    hold on



    set(gca, 'FontSize', fontsize );
    fig = gcf;
    fig.Color = [1 1 1];
    box('off');
    legend({'u_{wf}','lb_{wf}', 'ub_{wf}', 'u_{bf}'},'Location','northeast')

end


