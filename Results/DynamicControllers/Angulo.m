function [ErrAng] = Angulo(ErrAng)
%  Funcion "Angulo" que restringe el angulo de [0 : pi] y [-pi : 0]

% 1) ARGUMENTOS DE ENTRADA
  % a) ErrAng ----> ángulo en radianes
     
% 2) ARGUMENTOS DE SALIDA
  % a) ErrAng ----> ángulo de entrada limitado entre [0 : pi] y [-pi : 0]

if ErrAng>=1.00*pi
    while ErrAng>=1*pi
    ErrAng=ErrAng-2*pi;
    end
    return
end

if ErrAng<=-1.00*pi
    while ErrAng<=-1*pi
    ErrAng=ErrAng+2*pi;
    end
    return
end    
    ErrAng=ErrAng;
return

