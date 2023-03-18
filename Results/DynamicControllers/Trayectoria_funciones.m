function [] = Trayectoria_funciones(n,t)

% Funcion "Trayectoria_funciones" define la trayectoria deseada para el extremo operativo del manipulador aereo

% 1) ARGUMENTOS DE ENTRADA
  % a) n ----> Selector de trayectoria:
       % 1 --> Churo
       % 2 --> Seno
       % 3 --> 8
       % 4 --> Silla de montar
  % b) t ----> Vector de tiempo uniforme
      
% 2) ARGUMENTOS DE SALIDA
  % a) Posiciones DESEADAS:
      % xd ----> Posición en X 
      % yd ----> Posición en Y 
      % zd ----> Posición en Z 
      % psid --> Posición angular YAW 
  % b) Velocidades DESEADAS:
      % xd_p ----> Velocidad lineal X 
      % yd_p ----> Velocidad lineal Y 
      % zd_p ----> Velocidad lineal Z 
      % psid_p --> Velocidad angular YAW    
      
%******************************************************************************************************************
%********************************************* TRAYECTORIAS *******************************************************
%******************************************************************************************************************

% 1) Variables globales 
global xd yd zd xd_p yd_p zd_p xd_pp yd_pp zd_pp psid psid_p

% 2) Seleccion de trayectoria deseada (POSICIÓN y VELOCIDADES)
     switch n
   % a)Trayectoria Churo     
        case 1   
            xd= 0.06 *t.* cos(0.2*t) ;   
            xd_p= 0.06 * cos(0.2*t) + 0.02 *0.2 *t.* (-sin(0.2*t));
            xd_pp= -0.06 * sin(0.2*t) + 0.02 *0.2 * (-sin(0.2*t)) - 0.02 * 0.2 * 0.2 * t.* cos(0.2*t);
            yd= 0.06 *t.* sin (0.2 * t);  
            yd_p= 0.06 * sin(0.2*t) + 0.02 *0.2*t.* cos(0.2*t);
            yd_pp= 0.06 * 0.2 * cos(0.2*t) + 0.02 *0.2* cos(0.2*t) - 0.02 * 0.2 * 0.2 *t.* sin(0.2*t);
            zd= 1 * sin (0.3 * t) + 10 ;
            zd_p= 1 * 0.3* cos(0.3*t);
            zd_pp= -1 * 0.3 * 0.3 * sin(0.3*t); 
   % b) Trayectoria seno   
        case 2 
            yd= 5 * sin(0.1*t) + 2;           yd_p= 5*0.1 * cos(0.1*t);         yd_pp= -5* 0.1* 0.1* sin(0.1*t);
            xd= 0.1*t;                        xd_p= 0.1* ones(1,length(t));     xd_pp= 0* ones(1,length(t));
            zd= 2 * ones(1,length(t)) +4;     zd_p= 0 * ones(1,length(t)); 
   % c) Trayectoria de un 8 
        case 3
            xd = 5 * sin(4*0.04*t)+0.1;         xd_p = 4*5*0.04*cos(4*0.04*t);     xd_pp =4*4*-5*0.04*0.04*sin(4*0.04*t);
            yd = 5 * sin(4*0.08*t)+0.1;         yd_p = 4*5*0.08*cos(4*0.08*t);     yd_pp = 4*4*-5*0.08*0.08*sin(4*0.08*t);               
            zd = 1 * sin (0.08 * t) +2 ;    zd_p = 0.08*cos(0.08*t);
   % d) Trayectoria Silla de Montar
        case 4  
            xd= 5 * cos(0.05*t) + 5;                xd_p=-0.25*sin(0.05*t);           xd_pp=-0.0125*cos(0.05*t);
            yd= 5 * sin (0.05 * t) ;                yd_p=0.25*cos(0.05*t);            yd_pp=-0.0125*sin(0.05*t);
            zd= 1 * sin (0.3 * t) +18 ;           zd_p=0.3*cos(0.3*t);              zd_pp=-0.09*sin(0.3*t);
   % e) Otra opcion
        otherwise
            disp("Ninuna de las anteriores");
     end

% 3) Cálculo de orientación 
     psid= 1*(atan2(yd_p,xd_p));
     psid_p = 1*(1./((yd_p./xd_p).^2+1)).*((yd_pp.*xd_p-yd_p.*xd_pp)./xd_p.^2);
     psid_p(1)=0;

end