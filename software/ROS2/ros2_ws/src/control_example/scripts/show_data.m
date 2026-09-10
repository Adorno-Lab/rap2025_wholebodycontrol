
close all
clc

h1 = figure;
set(h1, 'DefaultTextFontSize', 10);
set(h1, 'DefaultAxesFontSize', 10); % [pt]
set(h1, 'DefaultAxesFontName', 'mwa_cmr10');
set(h1, 'DefaultTextFontName', 'mwa_cmr10');
set(h1, 'Units', 'centimeters');
pos = get(h1, 'Position');
pos(3) = 2*20; % Select the width of the figure in [cm] 17
pos(4) = 3*10; % Select the height of the figure in [cm] 6
set(h1, 'Position', pos);
set(h1, 'PaperType', 'a4letter');
set(h1,'PaperPositionMode','auto')
set(h1, 'Renderer', 'Painters');

w = 1;
fontsize = 15;


dist = distances';
safe_dist_list =safe_distances';
safe_dist = safe_dist_list(:,1);
vfi_buffer_list = vfi_buffers';
vfi_buffer = vfi_buffer_list(:,1);
[p, ns] = size(dist);

t = linspace(0, ns*T, ns);
tline = linspace(0, ns, 10);
tags_cell = cellstr(tags);

for i=1:p
        subplot(4,5,i);
            
        plot(dist(i,:), 'k','LineWidth',w);
        hold on
        if any(strcmp(tags_cell{i}, 'C2'))
            plot(tline,(deg2rad(safe_dist(i)) - deg2rad(vfi_buffer(i)))*ones(length(tline)), '--b', 'LineWidth',1);
            hold on
            plot(tline,deg2rad(safe_dist(i))*ones(length(tline)), '--r', 'LineWidth',1);
        else
            plot(tline,(safe_dist(i) + vfi_buffer(i))*ones(length(tline)), '--b', 'LineWidth',1);
            hold on
            plot(tline,safe_dist(i)*ones(length(tline)), '--r', 'LineWidth',1);
        end
        
    %end
    set(gca, 'FontSize', fontsize );
    fig = gcf;
    fig.Color = [1 1 1];
    box('off');
    axis([0 inf 0 inf]);
    title(['d - ' tags_cell{i}], 'Interpreter', 'latex')
end


h2 = figure;
set(h2, 'DefaultTextFontSize', 10);
set(h2, 'DefaultAxesFontSize', 10); % [pt]
set(h2, 'DefaultAxesFontName', 'mwa_cmr10');
set(h2, 'DefaultTextFontName', 'mwa_cmr10');
set(h2, 'Units', 'centimeters');
pos = get(h2, 'Position');
pos(3) = 2*20; % Select the width of the figure in [cm] 17
pos(4) = 3*10; % Select the height of the figure in [cm] 6
set(h2, 'Position', pos);
set(h2, 'PaperType', 'a4letter');
set(h2,'PaperPositionMode','auto')
set(h2, 'Renderer', 'Painters');

w = 1;
fontsize = 15;

u_q = u_qdot';
q_log = q';


for i=1:12
    subplot(2,6,i);
    plot(u_q(i,:),'b', 'LineWidth',w);
    hold on
    plot(tline,q_dot_min(i)*ones(length(tline)), '--r', 'LineWidth',1);
    hold on
    plot(tline,q_dot_max(i)*ones(length(tline)), '--r', 'LineWidth',1);

    set(gca, 'FontSize', fontsize );
    fig = gcf;
    fig.Color = [1 1 1];
    box('off');
    title(['$u_{\dot{q}_{', num2str(i), '}}$'], 'Interpreter', 'latex')
end


    h3 = figure;
    set(h3, 'DefaultTextFontSize', 10);
    set(h3, 'DefaultAxesFontSize', 10); % [pt]
    set(h3, 'DefaultAxesFontName', 'mwa_cmr10');
    set(h3, 'DefaultTextFontName', 'mwa_cmr10');
    set(h3, 'Units', 'centimeters');
    pos = get(h3, 'Position');
    pos(3) = 2*20; % Select the width of the figure in [cm] 17
    pos(4) = 3*10; % Select the height of the figure in [cm] 6
    set(h3, 'Position', pos);
    set(h3, 'PaperType', 'a4letter');
    set(h3,'PaperPositionMode','auto')
    set(h3, 'Renderer', 'Painters');
    
    w = 1;
    fontsize = 15;

    u_t = u';
    u_s = u_t(13:end,:);
    smax_list = smax';
    for i=1:30
        subplot(5,6,i);
        plot(u_s(i,:),'b', 'LineWidth',w);
        hold on
        plot(smax_list(i,:), '--r', 'LineWidth',1);
        set(gca, 'FontSize', fontsize );
        fig = gcf;
        fig.Color = [1 1 1];
        box('off');
        title(['$u_{\dot{s}_{', num2str(i), '}}$'], 'Interpreter', 'latex')
    end


 h4 = figure;
set(h4, 'DefaultTextFontSize', 10);
set(h4, 'DefaultAxesFontSize', 10); % [pt]
set(h4, 'DefaultAxesFontName', 'mwa_cmr10');
set(h4, 'DefaultTextFontName', 'mwa_cmr10');
set(h4, 'Units', 'centimeters');
pos = get(h4, 'Position');
pos(3) = 2*20; % Select the width of the figure in [cm] 17
pos(4) = 3*10; % Select the height of the figure in [cm] 6
set(h4, 'Position', pos);
set(h4, 'PaperType', 'a4letter');
set(h4,'PaperPositionMode','auto')
set(h4, 'Renderer', 'Painters');


q_list = q_log;
q_arm = q_list(9:end, :);
q_arm_min = q_min(9:end);
q_arm_max = q_max(9:end);

for i=1:6
    subplot(2,3,i);
    plot(q_arm(i,:),'b', 'LineWidth',1);
    hold on
    plot(tline, q_arm_min(i)*ones(length(tline)), '--r', 'LineWidth',1);
    hold on
    plot(tline, q_arm_max(i)*ones(length(tline)), '--r', 'LineWidth',1);

 
    hold on
    plot(tline, (q_arm_max(i)-q_arm_buffer(i))*ones(length(tline)), '--m', 'LineWidth',1);
    hold on
    plot(tline, (q_arm_min(i)+q_arm_buffer(i))*ones(length(tline)), '--m', 'LineWidth',1);



    set(gca, 'FontSize', fontsize );
    fig = gcf;
    fig.Color = [1 1 1];
    box('off');
    title(['$q_{', num2str(i), '}$'], 'Interpreter', 'latex')
end
