%******************************************************************************************************************
%************************************ SEGUIMIENTO DE TRAYECTORIA **************************************************
%************************************* UAV *****************************************************
%******************************************************************************************************************
clc; clear all; close all; warning off % Inicializacion
 
ts = 0.1;       % Tiempo de muestreo
tfin = 120;      % Tiempo de simulación
t = 0:ts:tfin;

% Variables definidas por la TRAYECTORIA y VELOCIDADES deseadas
global xd yd zd xd_p yd_p zd_p psid psid_p 

% 1) Condiciones iniciales del manipulador aéreo
     % a) Posiciones iniciales del UAV
       xu(1) = 0; 
       yu(1) = 0; 
       zu(1) = 1; 
       psi(1)= 0;
     

% 2) Seleccion de trayectoria deseada del manipulador móvil
   % 1 ---> Churo
   % 2 ---> Seno
   % 3 ---> 8
   % 4 ---> Silla de montar
     Trayectoria_funciones(3,t); 

disp('Empieza el programa')

%******************************************************************************************************************
%***************************************** CONTROLADOR ***********************************************************
%*****************************************************************************************************************

for k=1:length(t)
    tic
  
% 1) LEY DE CONTROL DEL MANIPULADOR AÉREO 
     
     % a) Función que contiene la ley de control del manipulador aéreo
       % LEY DE CONTROL
    
    respuesta_uav = Solo_UAV(xd_p(k),yd_p(k),zd_p(k),psid_p(k),xd(k),yd(k),zd(k),psid(k),xu(k),yu(k),zu(k),psi(k)); 
    ul(k) = respuesta_uav(1);
    um(k) = respuesta_uav(2);
    un(k) = respuesta_uav(3);
    w(k)  = respuesta_uav(4); %rad
       
% 2) ROBOT MANIPULADOR AÉREO
    
    % a) Cinemática del UAV 
      xu_p(k) = ul(k) * cos(psi(k)) - um(k) * sin(psi(k));
      yu_p(k) = ul(k) * sin(psi(k)) + um(k) * cos(psi(k));
      zu_p(k) = un(k);
    
    % b) Método Euler Integración numerica de las velocidades del UAV
      xu(k+1) = xu_p(k)*ts + xu(k);
      yu(k+1) = yu_p(k)*ts + yu(k);
      zu(k+1) = zu_p(k)*ts + zu(k);      
      psi(k+1) = Angulo(w(k)*ts + psi(k));

% 3) Tiempo de máquina   
     dt(k) = toc;
end
disp('Fin de los cálculos')

%******************************************************************************************************************
%********************************* ANIMACIÓN SEGUIMIENTO DE TRAYECTORIA ******************************************
%% ******************************************************************************************************************
disp('Simulación RUN')

% 1) Parámetros del cuadro de animacion
  figure(1)
  axis equal
  view(-15,15) % Angulo de vista
  cameratoolbar
  title ("Simulación")
  
% 2) Configura escala y color del UAV
 Drone_Parameters(0.02);
 H1 = Drone_Plot_3D(xu(1),yu(1),zu(1),0,0,psi(1));hold on
 

% 3) Dimensiones del brazo robótico
  

% 4) Dibujo inicial del manipulador aéreo

  % a) Gráfica inicial del brazo
  
  
  % b) Gráfica inicial del UAV  

  
  % c) Gráfica de la trayectoria deseada
  plot3(xd,yd,zd,'--')
  xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
  
% 5) Simulación de movimiento del manipulador aéreo
  for k=1:4:length(t)  
  % a) Eliminas los dibujos anteriores del manipulador aéreo
    delete(H1);
    H1 = Drone_Plot_3D(xu(k),yu(k),zu(k),0,0,psi(k)); hold on
  % b) Gráfica la posición deseada vs actual en cada frame
    plot3(xu(1:k),yu(1:k),zu(1:k),'r')
    hold on
    plot3(xd(1:k),yd(1:k),zd(1:k),'b')
    
  % c) Grafica de brazo robótico
    
    
  % d) Gráfica del UAV
    
  
  pause(0.1)
  end
%%
%******************************************************************************************************************
%********************************************* GR�?FICAS ***********************************************************
%% ****************************************************************************************************************

% 1) Igualar columnas de los vectores creados
  xu(:,end)=[];
  yu(:,end)=[];
  zu(:,end)=[];
  psi(:,end)=[];
  
% 2) Cálculos del Error
  figure(2)
  hxe= xd - xu;
  hye= yd - yu;
  hze= zd - zu;
  psie= Angulo(psid-psi);
  plot(hxe), hold on, grid on
  plot(hye)
  plot(hze)
  plot(psie)
  legend("hxe","hye","hze","psie")
  title ("Errores de posición")
  
% 3) Posiciones deseadas vs posiciones reales del extremo operativo del manipulador aéreo
  figure(3)
  
  subplot(4,1,1)
  plot(xd)
  hold on
  plot(xu)
  legend("xd","hx")
  ylabel('x [m]'); xlabel('s [ms]');
  title ("Posiciones deseadas y reales del extremo operativo del manipulador aéreo")
  
  subplot(4,1,2)
  plot(yd)
  hold on
  plot(yu)
  legend("yd","hy")
  ylabel('y [m]'); xlabel('s [ms]');

  subplot(4,1,3)
  plot(zd)
  hold on
  plot(zu)
  grid on
  legend("zd","hz")
  ylabel('z [m]'); xlabel('s [ms]');

  subplot(4,1,4)
  plot(Angulo(psid))
  hold on
  plot(psi)
  legend("psid","psi")
  ylabel('psi [rad]'); xlabel('s [ms]');
  