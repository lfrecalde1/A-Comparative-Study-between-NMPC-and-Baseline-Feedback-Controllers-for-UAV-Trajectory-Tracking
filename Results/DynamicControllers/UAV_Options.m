function [uav_Sub,uav_Pub,uav_Pub_Msg]=UAV_Options(Topic_UAVSub,Topic_UAV_Pub,Msg_UAV_Pub)

uav_Sub = rossubscriber(Topic_UAVSub);
[uav_Pub,uav_Pub_Msg] = rospublisher(Topic_UAV_Pub,Msg_UAV_Pub);

end