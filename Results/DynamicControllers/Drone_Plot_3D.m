function Drone_Graph=Drone_Plot_3D(x ,y ,z ,psi, theta, phi)

global Drone
if nargin < 7
    
   color1 = [0.15 0.15 0.1];%[0.01 0.2 0.3];%[0 0 0.25]
   color2 = [0.1 0.2 0.3];
   color3 = [0.87 0.9 0.9];%[1 1 1]
   color4 = 'y';
end

%Matriz de Rotación
% RotX = [1 0 0; 0 cos(X(4)) -sin(X(4)); 0 sin(X(4)) cos(X(4))];
% RotY = [cos(X(5)) 0 sin(X(5)); 0 1 0; -sin(X(5)) 0 cos(X(5))];
% RotZ = [cos(X(6)) -sin(X(6)) 0; sin(X(6)) cos(X(6)) 0; 0 0 1];

RotX = [1 0 0; 0 cos(psi) -sin(psi); 0 sin(psi) cos(psi)];
RotY = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
RotZ = [cos(phi) -sin(phi) 0; sin(phi) cos(phi) 0; 0 0 1];

Rot = RotZ*RotY*RotX;

%Estructura
for i = 1:length(Drone.Estructura)
    rDrone = Rot * Drone.Estructura{i};
    Drone_Graph(i) = patch(rDrone(1,:)+x,rDrone(2,:)+y,rDrone(3,:)+z,color1);
end

tam = i;

%Hélice 1
for i = 1:length(Drone.Helice1)
    rDrone = Rot * Drone.Helice1{i};    
    Drone_Graph(tam+i) = patch(rDrone(1,:)+x,rDrone(2,:)+y,rDrone(3,:)+z,color2);
end

tam = tam + i;

%Hélice 2
for i = 1:length(Drone.Helice2)
    rDrone = Rot * Drone.Helice2{i};    
    Drone_Graph(tam+i) = patch(rDrone(1,:)+x,rDrone(2,:)+y,rDrone(3,:)+z,color2);
end

tam = tam + i;

%Hélice 3
for i = 1:length(Drone.Helice3)
    rDrone = Rot * Drone.Helice3{i};    
    Drone_Graph(tam+i) = patch(rDrone(1,:)+x,rDrone(2,:)+y,rDrone(3,:)+z,color3);
end

tam = tam + i;

%Hélice 4
for i = 1:length(Drone.Helice4)
    rDrone = Rot * Drone.Helice4{i};    
    Drone_Graph(tam+i) = patch(rDrone(1,:)+x,rDrone(2,:)+y,rDrone(3,:)+z,color3);
end

tam = tam + i;

%Soportes
for i = 1:length(Drone.Soportes)
    rDrone = Rot * Drone.Soportes{i};    
    Drone_Graph(tam+i) = patch(rDrone(1,:)+x,rDrone(2,:)+y,rDrone(3,:)+z,color4);
end

tam = tam + i;

% %Camara
% for i = 1:length(Drone.Camara)
%     rDrone = Rot * Drone.Camara{i};    
%     Drone_Graph(tam+i) = patch(rDrone(1,:)+x,rDrone(2,:)+y,rDrone(3,:)+z,[0 0 0]);
% end