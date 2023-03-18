function [xp] = Adaptation_law(Y,ve,x)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
K = 0.5*eye(19);
G = 0.1*eye(19);
xp = inv(K)*Y'*ve;
end

