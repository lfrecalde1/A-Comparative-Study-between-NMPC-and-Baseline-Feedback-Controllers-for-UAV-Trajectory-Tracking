%******************************************************************************************************************
%************************************ SEGUIMIENTO DE TRAYECTORIA **************************************************
%************************************* ROBOT MANIPULADOR AÉREO *****************************************************
%******************************************************************************************************************
clc; clear all; close all; warning off % Inicializacion

ts = 0.1;       % Tiempo de muestreo
tfin = 400;      % Tiempo de simulación
t = 0:ts:tfin;
a=0;
b=0;
L=[a;b];

%% Variables definidas por la TRAYECTORIA y VELOCIDADES deseadas
[xd, yd, zd, psid, xdp, ydp, zdp, psidp] = Trayectorias(3,t);
%% GENERALIZED DESIRED SIGNALS
hd = [xd; yd; zd; psid];
hdp = [xdp;ydp;zdp;psidp];

%% a) Posiciones iniciales del UAV
xu(1) = 0; 
yu(1) = 0; 
zu(1) = 1; 
psi(1)= 0;
h=[xu(1);yu(1);zu(1);psi(1)];

%% Matriz A & B del sistema dinamico DMD Offline (Continuo)
load("A_B_values.mat"); 

%% Values init for DMD ONLINE (Discreto)
load("G&P_DMDonline_values_init.mat");
Ae = G(:,1:4);
Be = G(:,5:end);

%% Velocidad inicial real del UAV
v_real = [0;0;0;0];
v_estimate1 = v_real(:,1);
sample = 0;
A_E_P = reshape(Ae,16,1)
B_E_P = reshape(Be,16,1) 

%% Windowed DMD Online Value
m=6;

%******************************************************************************************************************
%***************************************** CONTROLADOR ***********************************************************
%*****************************************************************************************************************
disp('Empieza el programa')
for k=1:length(t)-1
tic
%% 1) LEY DE CONTROL
vc(:,k) = Vc_UAV(hdp(:,k),hd(:,k),xu(k),yu(k),zu(k),psi(k)); 
ul(k)=vc(1,k); um(k)=vc(2,k); un(k)=vc(3,k); w(k)=vc(4,k);

%% DYAMIC COMPENSATION
if k >1
    vc_k = [ul(k);um(k);un(k);w(k)];
    vc_k_1 = [ul(k-1);um(k-1);un(k-1);w(k-1)];
    [vref_c] = dynamic_compensation_k(vc_k, vc_k_1, v_real(:,k-1), Ae, Be, 0.9, ts);
else
    vref_c = [0;0;0;0];

end
  %  compensacion_k(:,k) = vref_c;
%% 2) DINAMICA DEL UAV (VELOCIDAD Y POSICION)
v_real(:, k+1) = DMD_dymamic_system(A,B,v_real(:,k), vc(:,k),ts);
% Integracion numerica metodo Runge-Kutta 
h(:,k+1) = h(:,k)+ UAV_RK4(h(:,k),v_real(:,k+1),ts);
xu(k+1) = h(1,k+1);
yu(k+1) = h(2,k+1);
zu(k+1) = h(3,k+1);      
psi(k+1) = Angulo(h(4,k));
%% DMD ONLINE
v_estimate1(:, k+1) = Ae*v_estimate1(:,k)+ Be*vc(:,k);

if sample >= m
    [Ae,Be,P,G] = DMD_Online(m,v_estimate1,vc,v_real,P,G,k);
    sample = 0;   
end
sample = sample + 1;
A_E_P(:,k+1) = reshape(Ae,16,1);
B_E_P(:,k+1) = reshape(Be,16,1);  
    
%% Perturvacion
minimo =  -0.0199;
maximo =   0.0198;
noise(:,k)  =  (maximo-minimo) .* rand(4,1) + minimo;
A(1,:) = A(1,:) + noise(:,k)';
A(2,:) = A(2,:) + 1.1*noise(:,k)';
A(3,:) = A(3,:) + (0.3)*noise(:,k)';
A(4,:) = A(4,:) + (-1.2)*noise(:,k)';
B(1,:) = B(1,:) + 0.3*noise(:,k)';
B(2,:) = B(2,:) + -(0.3)*noise(:,k)';
B(3,:) = B(3,:) + (0.05)*noise(:,k)';
B(4,:) = B(4,:) + (-0.1)*noise(:,k)';

%% 3) Tiempo de máquina   
dt(k) = toc;

end
disp('Fin de los calculos')

%*************************************************************************%
%**************ANIMACION SEGUIMIENTO DE TRAYECTORIA **********************%
%% ***********************************************************************%
disp('Animacion RUN')

% 1) Parámetros del cuadro de animacion
figure(1)
axis equal
view(-15,15) % Angulo de vista
cameratoolbar
title ("Simulacion")

% 2) Configura escala y color del UAV
Drone_Parameters(0.02);
H1 = Drone_Plot_3D(xu(1),yu(1),zu(1),0,0,psi(1));hold on


% c) Gráfica de la trayectoria deseada
plot3(xd,yd,zd,'--')
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');

% 5) Simulación de movimiento del manipulador aéreo
for k=1:200:length(t)  
% a) Eliminas los dibujos anteriores del manipulador aéreo
delete(H1);
H1 = Drone_Plot_3D(xu(k),yu(k),zu(k),0,0,psi(k)); hold on
% b) Gráfica la posición deseada vs actual en cada frame
plot3(xu(1:k),yu(1:k),zu(1:k),'r')
hold on
plot3(xd(1:k),yd(1:k),zd(1:k),'b')

pause(0.1)
end

disp('FIN Simulación RUN')  

%%
%******************************************************************************************************************
%********************************************* GR�?FICAS ***********************************************************
%% ****************************************************************************************************************


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

% 3) Posiciones deseadas vs posiciones reales del extremo operativo del manipulador aéreo
figure(4)


plot(vc(1,20:end))
hold on
plot(v_real(1,20:end))
hold on
plot(v_estimate1(1,20:end))
legend("ulc","ul","ul_{est}")
ylabel('x [m/s]'); xlabel('s [ms]');
title ("Posiciones deseadas y reales del extremo operativo del manipulador aéreo")

figure(5)
plot(vc(2,20:end))
hold on
plot(v_real(2,20:end))
hold on
plot(v_estimate1(2,20:end))
legend("umc","um","um_{est}")
ylabel('y [m/s]'); xlabel('s [ms]');

figure(6)
plot(vc(3,20:end))
hold on
plot(v_real(3,20:end))
hold on
plot(v_estimate1(3,20:end))
legend("unc","un","un_{est}")
ylabel('z [m/ms]'); xlabel('s [ms]');

figure(7)
plot(vc(4,20:end))
hold on
plot(v_real(4,20:end))
hold on
plot(v_estimate1(4,20:end))
legend("wc","w","w_{ref}")
ylabel('psi [rad/s]'); xlabel('s [ms]');

figure(8)
plot(A_E_P',"-",'linewidth',2)
 legend("wc","w","w_{ref}")
ylabel('psi [rad/s]'); xlabel('s [ms]');

figure(9)

plot(B_E_P',"-",'linewidth',2)
legend("wc","w","w_{ref}")
ylabel('psi [rad/s]'); xlabel('s [ms]');



  