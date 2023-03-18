%% Code to show the results of the Inverse kinematic controller

% Clean variables
clc, clear all, close all;
 
% Load variables of the system
load("Validation.mat");

% Figure propert%% Figures
lw = 1; % linewidth 1
lwV = 2; % linewidth 2
fontsizeLabel = 11; %11
fontsizeLegend = 11;
fontsizeTicks = 11;
fontsizeTitel = 11;
sizeX = 1300; % size figure
sizeY = 650; % size figure

% color propreties
c1 = [80, 81, 79]/255;
c2 = [244, 213, 141]/255;
c3 = [242, 95, 92]/255;
c4 = [112, 141, 129]/255;

C18 = [0 0 0];
c5 = [130, 37, 37]/255;
c6 = [205, 167, 37]/255;
c7 = [81, 115, 180]/255;

% color propreties
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


%% Figure 1
axes('Position',[0.6 0.79 .45 .17]);
%% Data generation
ul_vali = line(t_vali,ul_vali);
set(ul_vali, 'LineStyle', '-', 'Color', C11, 'LineWidth', 1.1*lw);
ul_ref_vali = line(t_vali(1:length(ul_ref_vali)),ul_ref_vali);
set(ul_ref_vali, 'LineStyle', '-.', 'Color', C9, 'LineWidth', 1.3*lw);
ulm_vali = line(t_vali,v_estimate_vali(1,1:length(t_vali)));
set(ulm_vali, 'LineStyle', '--', 'Color', C12, 'LineWidth', 1.1*lw);

%% Title of the image
hTitle_1 = title({'$\textrm{(a)}$'},'fontsize',12,'interpreter','latex','Color',C18);
%xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);
%ylabel('$[m/s]$','fontsize',8,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_1 = legend([ul_vali,ul_ref_vali, ulm_vali],{'$\mu_{l}$','$\mu_{lref}$','$\mu_{lm}$'},'fontsize',10,'interpreter','latex','Color',[255 255 255]/255,'Location','best','NumColumns',1,'TextColor','black');
 set(gca,'ticklabelinterpreter','latex',...
         'fontsize',1.1*fontsizeTicks)
%% Figure properties
ax_1 = gca;
ax_1.Box = 'on';
ax_1.BoxStyle = 'full';
ax_1.TickLength = [0.01;0.01];
ax_1.TickDirMode = 'auto';
ax_1.XTickLabel = [];
ax_1.YMinorTick = 'on';
ax_1.XMinorTick = 'on';
ax_1.XMinorGrid = 'on';
ax_1.YMinorGrid = 'on';
%ax_1.MinorGridColor = '#8f8f8f';
ax_1.MinorGridAlpha = 0.15;
ax_1.LineWidth = 0.8;
ax_1.XLim = [0 t_vali(end)];

%% Figure 2
axes('Position',[0.6 0.58 .45 .17]);
%% Data generation
um_vali = line(t_vali,um_vali);
set(um_vali, 'LineStyle', '-', 'Color', C3, 'LineWidth', 1.1*lw);
um_ref_vali = line(t_vali(1:length(um_ref_vali)),um_ref_vali);
set(um_ref_vali, 'LineStyle', '-.', 'Color', C9, 'LineWidth', 1.3*lw);
umm_vali = line(t_vali,v_estimate_vali(2,1:length(t_vali)));
set(umm_vali, 'LineStyle', '--', 'Color', C14, 'LineWidth', 1.1*lw);

% fig1_comps.p1 = ul_plot;
%% Title of the image
%hTitle_2 = title({'$\textrm{(a)}$'},'fontsize',14,'interpreter','latex','Color',C18);
%xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);
%ylabel('$[m/s]$','fontsize',8,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_2 = legend([um_vali,um_ref_vali, umm_vali],{'$\mu_{m}$','$\mu_{mref}$','$\mu_{mm}$'},'fontsize',10,'interpreter','latex','Color',[255 255 255]/255,'Location','best','NumColumns',1,'TextColor','black');
 set(gca,'ticklabelinterpreter','latex',...
         'fontsize',1*fontsizeTicks)
%% Figure properties
ax_2 = gca;
ax_2.Box = 'on';
ax_2.BoxStyle = 'full';
ax_2.TickLength = [0.01;0.01];
ax_2.TickDirMode = 'auto';
ax_2.XTickLabel = [];
ax_2.YMinorTick = 'on';
ax_2.XMinorTick = 'on';
ax_2.XMinorGrid = 'on';
ax_2.YMinorGrid = 'on';
%ax_1.MinorGridColor = '#8f8f8f';
ax_2.MinorGridAlpha = 0.15;
ax_2.LineWidth = 0.8;
ax_2.XLim = [0 t_vali(end)];

%% Figure 3
axes('Position',[0.6 0.37 .45 .17]);
%% Data generation
un_vali = line(t_vali,un_vali);
set(un_vali, 'LineStyle', '-', 'Color', C2, 'LineWidth', 1.1*lw);
un_ref_vali = line(t_vali(1:length(un_ref_vali)),un_ref_vali);
set(un_ref_vali, 'LineStyle', '-.', 'Color', C9, 'LineWidth', 1.3*lw);
unm_vali = line(t_vali,v_estimate_vali(3,1:length(t_vali)));
set(unm_vali, 'LineStyle', '--', 'Color', C15, 'LineWidth', 1.1*lw);

% fig1_comps.p1 = ul_plot;
%% Title of the image
%hTitle_2 = title({'$\textrm{(a)}$'},'fontsize',14,'interpreter','latex','Color',C18);
%xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);
%ylabel('$[m/s]$','fontsize',8,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_3 = legend([un_vali,un_ref_vali, unm_vali],{'$\mu_{n}$','$\mu_{nref}$','$\mu_{nm}$'},'fontsize',10,'interpreter','latex','Color',[255 255 255]/255,'Location','best','NumColumns',1,'TextColor','black');
 set(gca,'ticklabelinterpreter','latex',...
         'fontsize',1*fontsizeTicks)
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
ax_3.XLim = [0 t_vali(end)];

%% Figure 4
axes('Position',[0.52 0.16 .45 .17]);
%% Data generation
w_vali= line(t_vali,w_vali);
set(w_vali, 'LineStyle', '-', 'Color', C16, 'LineWidth', 1.1*lw);
w_ref_vali = line(t_vali(1:length(w_ref_vali)),w_ref_vali);
set(w_ref_vali, 'LineStyle', '-.', 'Color', C9, 'LineWidth', 1.3*lw);
w_m_vali = line(t_vali,v_estimate_vali(4,1:length(t_vali)));
set(w_m_vali, 'LineStyle', '--', 'Color', C17, 'LineWidth', 1.1*lw);

% fig1_comps.p1 = ul_plot;
%% Title of the image
xlabel('$\textrm{Time}[s]$','fontsize',8,'interpreter','latex','Color',C18);
%ylabel('$[rad/s]$','fontsize',8,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_4 = legend([w_vali,w_ref_vali, w_m_vali],{'$\omega$','$\omega_{ref}$','$\omega_{m}$'},'fontsize',10,'interpreter','latex','Color',[255 255 255]/255,'Location','best','NumColumns',1,'TextColor','black');
 set(gca,'ticklabelinterpreter','latex',...
         'fontsize',1*fontsizeTicks)
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
ax_4.XLim = [0 t_vali(end)];
% % 
set(gcf, 'Color', 'w'); % Sets axes background
export_fig Kinematics_results.pdf -q101

