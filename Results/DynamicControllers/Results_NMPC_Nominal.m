%% Code to show the results of the Inverse kinematic controller

% Clean variables
clc, clear all, close all;
 
% Load variables of the system
load("T_MPC_Din_UAV_Real.mat")

% Change variable size
h = h(:, 1:end-1);
h_p = h_p(:, 1:end-1);
v = v(:, 1:end-1);
hd = hd(:,1:length(h));
% Time definition 
t = t;

% Error definition
for k = 1:length(t)-N
    error_vector = hd(1:3, k)-h(1:3, k);
    error_NMPC(k) = norm(error_vector,2);
end

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

% Figure 1
axes('Position',[0.04 0.65 .35 .31]);
%% Colorbar
scatter(h(1, :),h(2, :),30,error_NMPC,'filled','MarkerFaceAlpha',0.7);
c = colorbar;
c.Label.Interpreter = 'latex';
c.Label.String = '$\textrm{Error}~||\tilde{\mathbf{\eta}}||$';

set(c,'TickLabelInterpreter','latex')

%% Desired Trajectory
hd_plot = line(hd(1,:),hd(2,:));
set(hd_plot, 'LineStyle', '--', 'Color', C8, 'LineWidth', 1.2*lw)

%% Real Trajectory
h_plot = line(h(1,:),h(2,:));
set(h_plot, 'LineStyle', '-', 'Color', C18, 'LineWidth', 1.2*lw)

%% Title of the image
hTitle_1 = title({'$\textrm{(a)}$'},'fontsize',14,'interpreter','latex','Color',C18);
%hXLabel_1 = xlabel('$x~[m]$','fontsize',9,'interpreter','latex', 'Color',C18);
hYLabel_1 = ylabel('$y~[m]$','fontsize',9,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_1 = legend([hd_plot, h_plot],{'$\mathbf{\eta}_d$','$\mathbf{\eta}$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'Location','best','NumColumns',1,'TextColor','black');
set(gca,'ticklabelinterpreter','latex',...
        'fontsize',1.3*fontsizeTicks)
% Figure properties
ax_1 = gca;
ax_1.Box = 'on';
ax_1.BoxStyle = 'full';
ax_1.TickLength = [0.005;0.005];
ax_1.TickDirMode = 'auto';
ax_1.YMinorTick = 'on';
ax_1.XMinorTick = 'on';
% ax_2.XTickLabel = [];
ax_1.XMinorGrid = 'on';
ax_1.YMinorGrid = 'on';
%ax_2.MinorGridColor = '#8f8f8f';
ax_1.MinorGridAlpha = 0.15;
ax_1.LineWidth = 0.8;
ax_1.XLim = [min(h(1,:))-0.2 max(h(1,:))+0.2];
ax_1.YLim = [min(h(2,:))-0.2 max(h(2,:))+0.2];

% Figure 2
axes('Position',[0.04 0.28 .35 .31]);
%% Colorbar
scatter(h(1, :),h(3, :),30,error_NMPC,'filled','MarkerFaceAlpha',0.7);
c = colorbar;
c.Label.Interpreter = 'latex';
c.Label.String = '$\textrm{Error}~||\tilde{\mathbf{\eta}}||$';

set(c,'TickLabelInterpreter','latex')

%% Desired Trajectory
hd_plot_2 = line(hd(1,:),hd(3,:));
set(hd_plot_2, 'LineStyle', '--', 'Color', C8, 'LineWidth', 1.2*lw)

%% Real Trajectory
h_plot_2 = line(h(1,:),h(3,:));
set(h_plot_2, 'LineStyle', '-', 'Color', C18, 'LineWidth', 1.2*lw)

%% Title of the image
hTitle_2 = title({'$\textrm{(b)}$'},'fontsize',14,'interpreter','latex','Color',C18);
hXLabel_2 = xlabel('$x~[m]$','fontsize',9,'interpreter','latex', 'Color',C18);
hYLabel_2 = ylabel('$z~[m]$','fontsize',9,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_2 = legend([hd_plot_2, h_plot_2],{'$\mathbf{\eta}_d$','$\mathbf{\eta}$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'Location','best','NumColumns',1,'TextColor','black');
set(gca,'ticklabelinterpreter','latex',...
        'fontsize',1.3*fontsizeTicks)
% Figure properties
ax_2 = gca;
ax_2.Box = 'on';
ax_2.BoxStyle = 'full';
ax_2.TickLength = [0.005;0.005];
ax_2.TickDirMode = 'auto';
ax_2.YMinorTick = 'on';
ax_2.XMinorTick = 'on';
% ax_2.XTickLabel = [];
ax_2.XMinorGrid = 'on';
ax_2.YMinorGrid = 'on';
%ax_2.MinorGridColor = '#8f8f8f';
ax_2.MinorGridAlpha = 0.15;
ax_2.LineWidth = 0.8;
ax_2.XLim = [min(h(1,:))-0.2 max(h(1,:))+0.2];
ax_2.YLim = [min(h(3,:))-0.2 max(h(3,:))+0.2];

axes('Position',[0.45 0.65 .54 .31]);
%% Data generation
error_x_plot = line(t(1:length(h)),hd(1,:)-h(1,:));
set(error_x_plot, 'LineStyle', '-', 'Color', C5, 'LineWidth', 1.1*lw);
error_y_plot = line(t(1:length(h)),hd(2,:)-h(2,:));
set(error_y_plot, 'LineStyle', '-', 'Color', C3, 'LineWidth', 1.1*lw);
error_z_plot = line(t(1:length(h)),hd(3,:)-h(3,:));
set(error_z_plot, 'LineStyle', '-', 'Color', c7, 'LineWidth', 1.1*lw);

% fig1_comps.p1 = ul_plot;
%% Title of the image
hTitle_3 = title({'$\textrm{(c)}$'},'fontsize',14,'interpreter','latex','Color',C18);
%xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);
ylabel('$\textrm{Control Error}~[m]$','fontsize',10,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_3 = legend([error_x_plot,error_y_plot, error_z_plot],{'$\tilde{\mathbf{\eta}}_x$','$\tilde{\mathbf{\eta}}_y$','$\tilde{\mathbf{\eta}}_z$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'Location','best','NumColumns',1,'TextColor','black');
 set(gca,'ticklabelinterpreter','latex',...
         'fontsize',1.3*fontsizeTicks)
%% Figure properties
ax_3 = gca;
ax_3.Box = 'on';
ax_3.BoxStyle = 'full';
ax_3.TickLength = [0.01;0.01];
ax_3.TickDirMode = 'auto';
ax_3.XTickLabel = [];
ax_3.YMinorTick = 'on';
ax_3.XMinorTick = 'on';
ax_3.XMinorGrid = 'on';
ax_3.YMinorGrid = 'on';
%ax_1.MinorGridColor = '#8f8f8f';
ax_3.MinorGridAlpha = 0.15;
ax_3.LineWidth = 0.8;
ax_3.XLim = [0 t(end)];
%% Zoom Plot
axes('Position',[0.70 0.85 .2 .1]);
index_zoom_1 = (t>=20) & (t<=30);
error_x_plot_zoom = line(t(index_zoom_1),hd(1,(index_zoom_1))-h(1,(index_zoom_1)));
error_y_plot_zoom = line(t(index_zoom_1),hd(2,(index_zoom_1))-h(2,(index_zoom_1)));
error_z_plot_zoom = line(t(index_zoom_1),hd(3,(index_zoom_1))-h(3,(index_zoom_1)));

set(error_x_plot_zoom, 'LineStyle', '-', 'Color', C5, 'LineWidth', lw*0.8);
set(error_y_plot_zoom, 'LineStyle', '-', 'Color', C3, 'LineWidth', lw*0.8);
set(error_z_plot_zoom, 'LineStyle', '-', 'Color', c7, 'LineWidth', lw*0.8);

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
% 
axes('Position',[0.45 0.28 .54 .31]);
%% Data generation
error_vx_plot = line(t(1:length(h)),vc(1,:));
set(error_vx_plot, 'LineStyle', '-', 'Color', C7, 'LineWidth', 1.3*lw);
error_vy_plot = line(t(1:length(h)),vc(2,:));
set(error_vy_plot, 'LineStyle', '-', 'Color', C14, 'LineWidth', 1.3*lw);
error_vz_plot = line(t(1:length(h)),vc(3,:));
set(error_vz_plot, 'LineStyle', '-', 'Color', C15, 'LineWidth', 1.3*lw);

% fig1_comps.p1 = ul_plot;
%% Title of the image
hTitle_4 = title({'$\textrm{(d)}$'},'fontsize',14,'interpreter','latex','Color',C18);
xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);
ylabel('$\textrm{Inputs}~[m/s]$','fontsize',10,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_4 = legend([error_vx_plot,error_vy_plot, error_vz_plot],{'${\mu}_{l_{ref}}$','${\mu}_{m_{ref}}$','${\mu}_{n_{ref}}$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'Location','best','NumColumns',1,'TextColor','black');
 set(gca,'ticklabelinterpreter','latex',...
         'fontsize',1.3*fontsizeTicks)
%% Figure properties
ax_4 = gca;
ax_4.Box = 'on';
ax_4.BoxStyle = 'full';
ax_4.TickLength = [0.01;0.01];
ax_4.TickDirMode = 'auto';
ax_4.YMinorTick = 'on';
ax_4.XMinorTick = 'on';
ax_4.XMinorGrid = 'on';
ax_4.YMinorGrid = 'on';
%ax_1.MinorGridColor = '#8f8f8f';
ax_4.MinorGridAlpha = 0.15;
ax_4.LineWidth = 0.8;
ax_4.XLim = [0 t(end)];
% 
set(gcf, 'Color', 'w'); % Sets axes background
export_fig MPC_Nominal.pdf -q101

%% RMS dynamics
RMSE_x_NMPC = sqrt(mean((error_vector(1,:)).^2));
RMSE_y_NMPC = sqrt(mean((error_vector(2,:)).^2));
RMSE_z_NMPC = sqrt(mean((error_vector(3,:)).^2));

save("NMPC_rmse.mat","RMSE_x_NMPC", "RMSE_y_NMPC", "RMSE_z_NMPC","error_NMPC", "t", "N");