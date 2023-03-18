


close all
clc
ts = 1/30;       % Tiempo de muestreo
tfin = 45;      % Tiempo de simulación
t = 0:ts:tfin;

mul =  10;
Q = 0.2;
 xd = 4 * sin(mul*0.04*t);         xd_p = 4*mul*0.04*cos(mul*0.04*t);     xd_pp = -4*mul*mul*0.04*0.04*sin(mul*0.04*t);
 yd = 4 * sin(mul*0.08*t);         yd_p = 4*mul*0.08*cos(mul*0.08*t);     yd_pp = -4*mul*mul*0.08*0.08*sin(mul*0.08*t);               
 zd = 1.5* sin (Q * t) +5 ;    
 
figure (1)
axis equal
% Angulo de vista

 
 plot3(xd,yd,zd)
 view(0,0) 