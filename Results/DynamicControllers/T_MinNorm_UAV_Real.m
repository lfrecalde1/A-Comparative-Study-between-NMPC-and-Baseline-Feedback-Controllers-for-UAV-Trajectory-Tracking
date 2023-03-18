%******************************************************************************************************************
%************************************ SEGUIMIENTO DE TRAYECTORIA **************************************************
%************************************* UAV *****************************************************
%******************************************************************************************************************
clc; clear all; close all; warning off % Inicializacion
ts = 1/30;       % Tiempo de muestreo
tfin = 45;      % Tiempo de simulación
t = 0:ts:tfin;
%% Configuracion ROS - ARM Sub
active = true;
Master = 'http://192.168.88.251:11311';
Local = '192.168.88.247';

Uav_Topic_Sub ='/dji_sdk/odometry';
Uav_Topic_Pub ='/m100/velocityControl';
Uav_Msg_Pub = 'geometry_msgs/Twist';

ROS_Options(Master,Local,active);
[UAV_Sub,Uav_Pub,Uav_Pub_Msg] = UAV_Options(Uav_Topic_Sub,Uav_Topic_Pub,Uav_Msg_Pub);

%% 1) PUBLISHER TOPICS & MSG ROS
u_ref = [0.0, 0, 0, 0];
send_velocities(Uav_Pub, Uav_Pub_Msg, u_ref);

%% 2) Suscriber TOPICS & MSG ROS
[odo] = odometry(UAV_Sub);
h(:,1) = odo(1:4);
h_p(:,1) = odo(5:8);

%% Variables definidas por la TRAYECTORIA y VELOCIDADES deseadas
[xd, yd, zd, psid, xdp, ydp, zdp, psidp] = Trayectorias(3,t);
hd = [xd;yd;zd;0*psid];
hd_p = [xdp;ydp;zdp;0*psidp];
                                                      
disp('Empieza el programa')

%******************************************************************************************************************
%***************************************** CONTROLADOR ***********************************************************
%*****************************************************************************************************************

for k=1:length(t)
    tic
%% 1) Controlador
    [vc(:,k),he(:,k)] = Solo_UAV(xdp(k),ydp(k),zdp(k),0*psidp(k),hd(:,k),h(:,k)); 
    
%% 2) Envia velocidades al UAV    
    send_velocities(Uav_Pub, Uav_Pub_Msg, vc(:,k));
%% 3) Recibir datos - UAV          
try
[odo] = odometry(UAV_Sub);
h(:,k+1) = odo(1:4);
h(4,k+1) = Angulo(h(4,k+1)); 
h_p(:,k+1) = odo(5:8);        
catch
h(:,k+1) = h(:,k);
h_p(:,k+1) = h_p(:,k);    
end

J = [cos(psi(k)) -sin(psi(k)) 0 0;
sin(psi(k)) cos(psi(k)) 0 0;
0 0 1 0;
0 0 0 1];
    
v(:,k+1) = pinv(J)*h_p(:,k);
%% 4) Tiempo de muestreo
while toc < ts
end
    
%% 5) Tiempo de Loop   
     dt(k) = toc;     
end
disp('Fin de los cálculos')
u_ref = [0, 0, 0, 0];
send_velocities(Uav_Pub, Uav_Pub_Msg, u_ref);
%******************************************************************************************************************
%********************************* ANIMACIÓN SEGUIMIENTO DE TRAYECTORIA ******************************************
%% ******************************************************************************************************************
save("T_MinNorma_UAV_Real.mat","hd","hd_p","h","h_p","v","vc");

%%

disp('Simulación RUN')

% 1) Parámetros del cuadro de animacion
  figure(1)
  axis equal
  view(-15,15) % Angulo de vista
  cameratoolbar
  title ("Simulación")
  
% 2) Configura escala y color del UAV
 Drone_Parameters(0.02);
 H1 = Drone_Plot_3D(h(1,1),h(2,1),h(3,1),0,0,h(4,1));hold on
 
% c) Gráfica de la trayectoria deseada
p1 = plot3(xd,yd,zd,'--');
p2 = plot3(0,0,0,'--');
  
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
  
% 5) Simulación de movimiento del manipulador aéreo
  for k=1:4:length(t)  
  % a) Eliminas los dibujos anteriores del manipulador aéreo
    delete(H1);
    delete(p1);
    delete(p2);
    H1 = Drone_Plot_3D(h(1,k),h(2,k),h(3,k),0,0,h(4,k)); hold on
  % b) Gráfica la posición deseada vs actual en cada frame
    p1 = plot3(h(1,1:k),h(2,1:k),h(3,1:k),'r');
    hold on
    p2 = plot3(hd(1,1:k),hd(2,1:k),hd(3,1:k),'b');
    
  % c) Grafica de brazo robótico
    
    
  % d) Gráfica del UAV
    
  
  pause(0.1)
  end
%%


%******************************************************************************************************************
%********************************************* GR�?FICAS ***********************************************************
%% ****************************************************************************************************************

% 1) Igualar columnas de los vectores creados
%   xu(:,end)=[];
%   yu(:,end)=[];
%   zu(:,end)=[];
%   psi(:,end)=[];
  
% 2) Cálculos del Error
  figure(2)
  hxe= he(1,1:end);
  hye= he(2,1:end);
  hze= he(3,1:end);
  psie= Angulo(hd(4,1:end)-h(4,1:end-1));
  plot(hxe), hold on, grid on
  plot(hye)
  plot(hze)
  plot(psie)
  legend("hxe","hye","hze","psie")
  title ("Errores de posición")
  
% 3) Posiciones deseadas vs posiciones reales del extremo operativo del manipulador aéreo
  figure(3)
  
  subplot(4,1,1)
  plot(hd(1,1:end))
  hold on
  plot(h(1,1:end))
  legend("xd","hx")
  ylabel('x [m]'); xlabel('s [ms]');
  title ("Posiciones deseadas y reales del extremo operativo del manipulador aéreo")
  
  subplot(4,1,2)
  plot(hd(2,1:end))
  hold on
  plot(h(2,1:end))
  legend("yd","hy")
  ylabel('y [m]'); xlabel('s [ms]');

  subplot(4,1,3)
  plot(hd(3,1:end))
  hold on
  plot(h(3,1:end))
  grid on
  legend("zd","hz")
  ylabel('z [m]'); xlabel('s [ms]');

  subplot(4,1,4)
  plot(hd(4,1:end))
  hold on
  plot(h(4,1:end))
  legend("psid","psi")
  ylabel('psi [rad]'); xlabel('s [ms]');
  
  figure(4)
  ul_e= hd_p(1,1:end) - v(1,1:size(hd_p(1,1:end),2));
  um_e= hd_p(2,1:end) - v(2,1:size(hd_p(1,1:end),2));
  un_e= hd_p(3,1:end) - v(3,1:size(hd_p(1,1:end),2));
  
  plot(ul_e), hold on, grid on
  plot(um_e)
  plot(un_e)
  legend("hxe","hye","hze","psie")
  title ("Errores de posición")