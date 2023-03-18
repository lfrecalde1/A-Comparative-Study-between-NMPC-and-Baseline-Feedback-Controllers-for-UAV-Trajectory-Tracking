%******************************************************************************************************************
%************************************ SEGUIMIENTO DE TRAYECTORIA **************************************************
%************************************* ROBOT MANIPULADOR AÉREO *****************************************************
%******************************************************************************************************************
clc; clear all; close all; warning off % Inicializacion
 
ts = 0.1;       % Tiempo de muestreo
tfin = 25;   % Tiempo de simulación
t = 0:ts:tfin;
load("chi_values.mat");
a =0;
b=0;
L=[a;b]
x = chi;

% Variables definidas por la TRAYECTORIA y VELOCIDADES deseadas
global xd yd zd xd_p yd_p zd_p psid psid_p 

% 1) Condiciones iniciales del manipulador aéreo
     % a) Posiciones iniciales del UAV
       xu(1) = 0; 
       yu(1) = 0; 
       zu(1) = 1; 
       psi(1)= 0;
     
       xuc(1) = 0; 
       yuc(1) = 0; 
       zuc(1) = 1; 
       psic(1)= 0;

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
u_real = [0;0;0;0];
h=[0;0;1;0]


for k=1:length(t)
    tic
  
% 1) LEY DE CONTROL DEL MANIPULADOR AÉREO 
     
     % a) Función que contiene la ley de control del manipulador aéreo
       % LEY DE CONTROL
      
      u_ref(:,k) = Solo_UAV(xd_p(k),yd_p(k),zd_p(k),psid_p(k),xd(k),yd(k),zd(k),psid(k),xu(k),yu(k),zu(k),psi(k)); 
      u_refc(:,k) = Solo_UAV(xd_p(k),yd_p(k),zd_p(k),psid_p(k),xd(k),yd(k),zd(k),psid(k),xuc(k),yuc(k),zuc(k),psic(k)); 
% 2) ROBOT MANIPULADOR AÉREO
      u_real(:, k+1) = system_dynamic(x, u_real(:,k), u_ref(:,k), psi(k), L,ts);
      ul(k) = u_real(1,k+1);
      um(k) = u_real(2,k+1);
      un(k) = u_real(3,k+1);
      w(k)  = u_real(4,k+1); %rad
      
          %% deficnion del vector de estados del sistema

      hp(:,k) = cinematicaUAV([xuc(k);yuc(k);zuc(k);psic(k)],u_refc(:,k));
      xuc(k+1) = hp(1,k)*ts + xuc(k);
      yuc(k+1) = hp(2,k)*ts + yuc(k);
      zuc(k+1) = hp(3,k)*ts + zuc(k);      
      psic(k+1) = Angulo(hp(4,k)*ts + psic(k));

      %% Integracion numerica metodo Runge-Kutta 
      h(:,k+1) = h(:,k)+ UAV_RK4(h(:,k),u_ref(:,k),ts);
      xu(k+1) = h(1,k+1);
      yu(k+1) = h(2,k+1);
      zu(k+1) = h(3,k+1);      
      psi(k+1) = Angulo(h(4,k+1));


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
  psic(:,end)=[];
  
% 2) Cálculos del Error
  figure(2)
  hxe= xd - xu;
  hye= yd - yu;
  hze= zd - zu;
  psie= Angulo(psid-psic);
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
  