function [x] = adaptacion(x, Y, ve, ts)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
xp = Adaptation_law(Y, ve,x);
x = x + xp*ts;
end

