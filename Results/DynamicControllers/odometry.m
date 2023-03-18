function [data] = odometry(odomSub)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here


% Read odometry values from ros
odomdata = receive(odomSub,0.1);  %(the second argument is a time-out in seconds).
%pose = odomdata.Pose.Pose;
%vel = odomdata.Twist.Twist;
%quat = pose.Orientation;
%angles = quat2eul([quat.W quat.X quat.Y quat.Z]);

data = odomdata.Axes;


end

