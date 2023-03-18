function  send_velocities(robot, velmsg, vd)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
%  0.1145;
value = 0.11;
vel_rad_s1 = vd(1);
rpm1 = (30*vel_rad_s1)/pi;
x1 = rpm1/value;

vel_rad_s2 = vd(2);
rpm2 = (30*vel_rad_s2)/pi;
x2 = rpm2/value;

% Send desired velocities to the robot Linear
vx = 0;
vy = 0;
vz = 0;

% Send desired velocities to the robot angular
wx = 0;
q2 = x2;
q1 = x1;

velmsg.Linear.X = vx;
velmsg.Linear.Y = vy;
velmsg.Linear.Z = vz;

velmsg.Angular.X = wx;
velmsg.Angular.Y = x2;
velmsg.Angular.Z = x1;


send(robot,velmsg);
end

