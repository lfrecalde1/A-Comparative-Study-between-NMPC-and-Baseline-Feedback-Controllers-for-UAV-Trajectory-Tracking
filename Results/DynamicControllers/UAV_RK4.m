function [st_next_RK4]=UAV_RK4(h,u_real,ts)

    k1 = cinematicaUAV(h,u_real);
    k2 = cinematicaUAV(h + ts/2*k1, u_real);% new
    k3 = cinematicaUAV(h + ts/2*k2, u_real); % new
    k4 = cinematicaUAV(h + ts*k3, u_real); % new
    st_next_RK4=(ts/6*(k1 +2*k2 +2*k3 +k4));
    
%     k1 = f(st, con);   % new 
%     k2 = f(st + ts/2*k1, con); % new
%     k3 = f(st + ts/2*k2, con); % new
%     k4 = f(st + ts*k3, con); % new
%     st_next_RK4=st +ts/6*(k1 +2*k2 +2*k3 +k4);
    
end