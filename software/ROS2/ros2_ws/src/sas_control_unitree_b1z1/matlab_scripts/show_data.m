

clear all
close all
clc

load("sas_log_2025_12_19_13_54_20.mat")
%load("/home/s55322jq/sas_log_2025_12_04_13_45_13.mat")



u_cmd = u';
ub_cmd = ub'; 

q_dot_min_body_frame = q_dot_min';
q_dot_max_body_frame = q_dot_max';

q_dot_min_inertial_frame = q_dot_min_inertial';
q_dot_max_inertial_frame = q_dot_max_inertial';


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




for i=1:3

    subplot(3,1,i)
    plot(u_cmd(i,:),"b",'LineWidth', 2) 
    hold on
    plot(q_dot_min_inertial_frame(i,:), "--b") 
    hold on
    plot(q_dot_max_inertial_frame(i,:), "--b")
    % 
    hold on
    plot(q_dot_min_body_frame(i,:), "--r", 'LineWidth', 2) 
    hold on
    plot(q_dot_max_body_frame(i,:), "--r", 'LineWidth', 2)
    hold on
    plot(ub_cmd(i,:),"r", 'LineWidth', 2) 
    title(['u_',num2str(i)])
    
    % 
    % 
    hold on


    %plot(0.1*robot_reached_region, "g", 'LineWidth', 4)

    set(gca, 'FontSize', fontsize );
    fig = gcf;
    fig.Color = [1 1 1];
    box('off');
    legend({'u_{wf}','lb_{wf}', 'ub_{wf}', 'lb_{bf}', 'ub_{bf}', 'u_{bf}'},'Location','northeast')

end


