function  dynamixel=rad2value(rad_s)

value = 0.11;
vel_rad_s1 = rad_s;
rpm1 = (30*vel_rad_s1)/pi;
dynamixel = round(rpm1/value);

end