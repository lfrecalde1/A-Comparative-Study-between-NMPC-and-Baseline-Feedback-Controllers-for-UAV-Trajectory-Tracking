function vref = dynamicComDMD_Offline(A,B,vcp, vc, v, k3, k4)

%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

%% Gain Matrices
K3 = k3*eye(size(v,1));
K4 = k4*eye(size(v,1));

%% Control error veclocity
ve = vc-v;
control = vcp + K3*tanh(inv(K3)*K4*ve);
vref = inv(B)*control-inv(B)*A*vc;
% vref = A*v+B*vc
end