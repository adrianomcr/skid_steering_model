function [vel] = compute_relative_vel(u,c,vx,vy,wz,r)
    
    %Compute the velocity of the contact point of the weel with respect to the floor
    Sw = [0 -wz; wz 0]; %skew-symetric matrix
    vel = [-r*u; 0]; %due to the wheel rotation speed
    vel = vel + Sw*c; %due to the robots angular velocity
    vel = vel + [vx; vy]; %due to the robots linear velocity
    
end