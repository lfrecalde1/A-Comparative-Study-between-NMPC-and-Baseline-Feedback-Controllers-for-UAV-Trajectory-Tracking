function [v] = system_dynamic(x, v, vref, psi, L, ts)
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here
vp = dynamic_func(x, v, vref, psi, L);
v = v +vp*ts;
end

