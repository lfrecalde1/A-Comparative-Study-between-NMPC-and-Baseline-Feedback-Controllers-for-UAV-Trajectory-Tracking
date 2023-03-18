function [VMref] = Vc_UAV(hdp,hd,hx,hy,hz,psi)

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

xd = hd(1);
yd = hd(2);
zd = hd(3);
psid = hd(4);

%3) Calculos del Error
  hxe= xd - hx;
  hye= yd - hy;
  hze= zd - hz;
  psie= Angulo(psid-psi);   
  he= [hxe hye hze psie];

% Constantes de ganancia ( ROS DJI_SDK)
K1 = diag(1.5*[1 1 1 2]); 
K2 = diag(0.6*[0.6 0.6 0.6 0.6]);   
D = diag([1 1 1 1 1 1 1]);

% 7) Ley de control completa,  solucion = [u omega qpunto1 qpunto2]    
  VMref = pinv(J)*(hdp+K1*tanh(K2*he'));
  
end