function [Omega_r,Omega_l] = open_loop_reference(t,a,b,r)

    %Provide an open loop command to the wheels rotation speeds
    Omega_r = 1.55;
    Omega_l = -1.55/-2;

%     %Use this to set v_x and omega_z based on the kinematic model
%     M = [0.5, 0.5; b/(2*(a^2+b^2)), -b/(2*(a^2+b^2))];
%     v = 0.5;
%     omega = 0.5;
%     Vde = M^(-1)*[v; omega];
%     Omega_r = Vde(1)/r;
%     Omega_l = Vde(2)/r;
 
end
