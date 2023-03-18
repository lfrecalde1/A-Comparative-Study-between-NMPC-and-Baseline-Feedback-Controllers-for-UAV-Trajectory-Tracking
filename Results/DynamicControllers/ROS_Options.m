function ROS_Options(Master,Local,active)


rosshutdown

if active
   setenv('ROS_MASTER_URI',Master)
   setenv('ROS_IP',Local) 
else
   disp("Ros Master Local")
end

rosinit

end