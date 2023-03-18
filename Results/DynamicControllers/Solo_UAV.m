function [VMref,he] = Solo_UAV(xd_p,yd_p,zd_p,psid_p,hd,h)

psi = h(4);

J11 = cos(psi);
J12 = -sin(psi);
J13 = 0;
J14 = 0;

J21 = sin(psi);
J22 = cos(psi);
J23 = 0;
J24 = 0;

J31 = 0;
J32 = 0;
J33 = 1;
J34 = 0;

J41 = 0;
J42 = 0;
J43 = 0;
J44 = 1;

J = [[J11 J12 J13 J14];[J21 J22 J23 J24];[J31 J32 J33 J34];[J41 J42 J43 J44]];
% J = [[J11 J12 J13 J14];[J21 J22 J23 J24];[J31 J32 J33 J34]];

%3) Calculos del Error

% xd = hd(1);
% yd = hd(2);
% zd = hd(3);
% psid = hd(4);
% 
% hx = h(1);
% hy = h(2);
% hz = h(3);
% psi = h(4);
% 
%   hxe= xd - hx;
%   hye= yd - hy;
%   hze= zd - hz;
%   psie= Angulo(psid-psi);   
%   he= [hxe ;hye ;hze ;psie];
  
 he = hd - h;
 he(4) = Angulo(he(4));
  
  %he = hd-h;
  %he(4) = Angulo(h(4));
% Constantes de ganancia ( ROS DJI_SDK)
K1 = diag(1.5*[1 2 3 1]); 
K2 = diag(0.5*[1 1 1 1]);   
D = diag([1 1 1 1 1 1 1]);

% 7) Ley de control completa,  solucion = [u omega qpunto1 qpunto2]    
 VMref = pinv(J)*([xd_p yd_p zd_p psid_p]'+K1*tanh(K2*he));
%    VMref = pinv(J)*(K1*tanh(K2*he'));
  
end