function [hp] = cinematicaUAV(h,u_real)

ul=u_real(1);
um=u_real(2);
un=u_real(3);
w =u_real(4);
xu_p = ul * cos(h(4)) - um * sin(h(4));
yu_p = ul * sin(h(4)) + um * cos(h(4));
zu_p = un;

hp = [xu_p;yu_p;zu_p;w];

end