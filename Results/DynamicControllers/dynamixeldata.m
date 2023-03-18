function [pos] = dynamixeldata(dynamixelSub)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here


dynadata = receive(dynamixelSub,0.1); 


pos = dynadata.Velocity;


end

