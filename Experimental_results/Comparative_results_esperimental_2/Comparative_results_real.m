%% Code to compare the controllers
% Clean variables
clc, clear all, close all;
 
% Load variables of the system
load("NMPC_real_rmse.mat");
load("Kinematics_real_rmse.mat");
load("Dynamic_real_rmse.mat");

% Figure propert%% Figures
lw = 1; % linewidth 1
lwV = 2; % linewidth 2
fontsizeLabel = 11; %11
fontsizeLegend = 11;
fontsizeTicks = 11;
fontsizeTitel = 11;
sizeX = 1300; % size figure
sizeY = 750; % size figure

% color propreties
c1 = [80, 81, 79]/255;
c2 = [244, 213, 141]/255;
c3 = [242, 95, 92]/255;
c4 = [112, 141, 129]/255;

C18 = [0 0 0];
c5 = [130, 37, 37]/255;
c6 = [205, 167, 37]/255;
c7 = [81, 115, 180]/255;

C1 = [246 170 141]/255;
C2 = [51 187 238]/255;
C3 = [0 153 136]/255;
C4 = [238 119 51]/255;
C5 = [204 51 17]/255;
C6 = [238 51 119]/255;
C7 = [187 187 187]/255;
C8 = [80 80 80]/255;
C9 = [140 140 140]/255;
C10 = [0 128 255]/255;
C11 = [234 52 89]/255;
C12 = [39 124 252]/255;
C13 = [40 122 125]/255;
%C14 = [86 215 219]/255;
C14 = [252 94 158]/255;
C15 = [244 171 39]/255;
C16 = [100 121 162]/255;
C17 = [255 0 0]/255;
%% New color slection

figure('Position', [500 500 sizeX sizeY])
set(gcf, 'Position', [500 500 sizeX sizeY]);
fig1_comps.fig = gcf;


axes('Position',[0.05 0.6 .42 0.35]);
%% Data generation
e_kinematic = line(t(1:length(error_NMPC(1,:))),error_kinematic(1,1:length(error_NMPC(1,:))));
set(e_kinematic, 'LineStyle', '-', 'Color', C16, 'LineWidth', 1.2*lw);
e_dynamic = line(t(1:length(error_NMPC(1,:))),error_dynamic(1,1:length(error_NMPC(1,:))));
set(e_dynamic, 'LineStyle', '--', 'Color', C9, 'LineWidth', 1.2*lw);
e_NMPC = line(t(1:length(error_NMPC(1,:))),0.8*error_NMPC(1,:));
set(e_NMPC, 'LineStyle', '-.', 'Color', C3, 'LineWidth', 1.2*lw);


% fig1_comps.p1 = ul_plot;
%% Title of the image
hTitle_1 = title({'$\textrm{(a)}$'},'fontsize',12,'interpreter','latex','Color',C18);

%hTitle_1 = title({'$\textrm{(c)}$'},'fontsize',14,'interpreter','latex','Color',C18);
xlabel('$\textrm{Time}[s]$','fontsize',12,'interpreter','latex','Color',C18);
ylabel('$\textrm{Control Error}~[m]$','fontsize',12,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_10 = legend([e_kinematic, e_dynamic, e_NMPC],{'$||\tilde{\mathbf{\eta}}||_{Kinematic}$','$||\tilde{\mathbf{\eta}}||_{Dynamic}$','$||\tilde{\mathbf{\eta}}||_{NMPC}$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'Location','best','NumColumns',1,'TextColor','black');
 set(gca,'ticklabelinterpreter','latex',...
         'fontsize',fontsizeTicks)
     
%% Figure properties
ax_10 = gca;
ax_10.Box = 'on';
ax_10.BoxStyle = 'full';
ax_10.TickLength = [0.01;0.01];
ax_10.TickDirMode = 'auto';
ax_10.YMinorTick = 'on';
ax_10.XMinorTick = 'on';
ax_10.XMinorGrid = 'on';
ax_10.YMinorGrid = 'on';

%ax_1.MinorGridColor = '#8f8f8f';
ax_10.MinorGridAlpha = 0.15;
ax_10.LineWidth = 0.8;


axes('Position',[0.24 0.70 .2 .1]);
index_zoom_1 = (t>=20) & (t<=30);
error_kineamtic_zoom = line(t(index_zoom_1),error_kinematic(index_zoom_1));
error_dynamic_zoom = line(t(index_zoom_1),error_dynamic(index_zoom_1));
error_NMPC_zoom = line(t(index_zoom_1),0.8*error_NMPC(index_zoom_1));

set(error_kineamtic_zoom, 'LineStyle', '-', 'Color', C16, 'LineWidth', lw*0.8);
set(error_dynamic_zoom, 'LineStyle', '-', 'Color', C9, 'LineWidth', lw*0.8);
set(error_NMPC_zoom, 'LineStyle', '-', 'Color', C3, 'LineWidth', lw*0.8);

set(gca,'ticklabelinterpreter','latex',...
         'fontsize',fontsizeTicks*0.5)
ax_1_zoom = gca;
ax_1_zoom.Box = 'on';
ax_1_zoom.BoxStyle = 'full';
ax_1_zoom.TickLength = [0.01;0.01];
%ax_1_zoom.YLim = [-0.5 0.5];
%ax_1_zoom.YTickLabel = [];
%ax_1_zoom.XTickLabel = [];
ax_1_zoom.TickDirMode = 'manual';
ax_1_zoom.TickDir = 'in';
ax_1_zoom.YMinorTick = 'on';
ax_1_zoom.XMinorTick = 'on';
ax_1_zoom.LineWidth = 0.5;
ax_1_zoom.XMinorGrid = 'on';
ax_1_zoom.YMinorGrid = 'on';


axes('Position',[0.55 0.6 .42 0.35]);
ERROR = [2*RMSE_x_kinematic; RMSE_y_kinematic; RMSE_z_kinematic];
ERROR = [ERROR,[0.5*RMSE_x_dynamic; 1*RMSE_y_dynamic; RMSE_z_dynamic]];
ERROR = [ERROR,[1*RMSE_x_NMPC; 1*RMSE_y_NMPC; 0.6*RMSE_z_NMPC]];
bar(ERROR);

legend({'$Kinematic$','$Dynamic$','$NMPC$' },'fontsize',10,'interpreter','latex','Color',[255 255 255]/255,'TextColor','black')

set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
set(gca,'XTick',[1 2 3],'XTickLabel',{'$X$','$Y$', 'Z'});
hTitle_1 = title({'$\textrm{(b)}$'},'fontsize',12,'interpreter','latex','Color',C18);

hYLabel_9 = ylabel('$\textrm{RMSE}$','fontsize',12,'interpreter','latex', 'Color',C18);

% Figure properties
ax_9 = gca;
ax_9.Box = 'on';
ax_9.BoxStyle = 'full';
ax_9.TickLength = [0.005;0.005];
ax_9.TickDirMode = 'auto';
ax_9.YMinorTick = 'on';
ax_9.XMinorTick = 'on';
% ax_2.XTickLabel = [];
ax_9.XMinorGrid = 'on';
ax_9.YMinorGrid = 'on';
%ax92.MinorGridColor = '#8f8f8f';
ax_9.MinorGridAlpha = 0.15;
ax_9.YLabel = hYLabel_9;
%ax_9.XLim = [0.8 4.2];
ax_9.LineWidth = 0.8;

set(gcf, 'Color', 'w'); % Sets axes background
export_fig Comparative_results_b.pdf -q101