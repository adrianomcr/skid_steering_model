

%% Animation

x = states(1,:); %position x
y = states(2,:); %position y
psi = states(3,:); %yaw angle

SHOW_FORCES = 0;

ws = [-3 3 -2 2 -0.1 2] + [-2 2 -2 2 0 0]*0; %workspace size

% Inclusion of pitch
H0 = eye(4);
ws(5) = -0.3;

H_cm = eye(4);
H_cm(1:2,4) = -CM(1:2);

figure(100)
plot3(0,0,0,'k')
xlabel('x')
ylabel('y')
zlabel('z')
axis equal
axis(ws)
grid on

hold on
%Plot floor
H_floor = [Rot('z',pi/2)*Rot('y',pi/2), [0;0;0]; 0 0 0 1];
plot_rectangular_face(H0*H_floor,[ws(2)-ws(1),ws(4)-ws(3)],[0 0 0]+0.9,0.7,'-')

%Plot body
H_body = [Rot('z',psi(1)), [x(1);y(1);r]; 0 0 0 1];
h_body = plot_block(H0*H_body*H_cm,size_body,[0 0 0]+0.5,0.8,'-');


width_wheel = 0.04;
number_faces = 10;
%Plot wheel 1
H_w10 = eye(4);
H_w10(1:3,1:3) = Rot('x',pi/2);
H_w10(1:3,4) = [a; -(b+0.02); 0];
H_w1 = H_body*H_cm*H_w10;
H_w1(1:3,1:3) = H_w1(1:3,1:3)*Rot('z',-wheels(1,1));
h_w1 = plot_cylinder(H0*H_w1,width_wheel,r,[0.7 0 0]+0.1,0.7,number_faces,'-');
%Plot wheel 2
H_w20 = eye(4);
H_w20(1:3,1:3) = Rot('x',pi/2);
H_w20(1:3,4) = [-a; -(b+0.02); 0];
H_w2 = H_body*H_cm*H_w20;
H_w2(1:3,1:3) = H_w2(1:3,1:3)*Rot('z',-wheels(2,1));
h_w2 = plot_cylinder(H0*H_w2,width_wheel,r,[0.7 0 0]+0.1,0.7,number_faces,'-');
%Plot wheel 3
H_w30 = eye(4);
H_w30(1:3,1:3) = Rot('x',pi/2);
H_w30(1:3,4) = [-a; (b+0.02); 0];
H_w3 = H_body*H_cm*H_w30;
H_w3(1:3,1:3) = H_w3(1:3,1:3)*Rot('z',-wheels(3,1));
h_w3 = plot_cylinder(H0*H_w3,width_wheel,r,[0.7 0 0]+0.1,0.7,number_faces,'-');
%Plot wheel 4
H_w40 = eye(4);
H_w40(1:3,1:3) = Rot('x',pi/2);
H_w40(1:3,4) = [a; (b+0.02); 0];
H_w4 = H_body*H_cm*H_w40;
H_w4(1:3,1:3) = H_w4(1:3,1:3)*Rot('z',-wheels(4,1));
h_w4 = plot_cylinder(H0*H_w4,width_wheel,r,[0.7 0 0]+0.1,0.7,number_faces,'-');


%Plot CM
cm_rot = H_body(1:3,1:3)*CM;
h_cm1 = plot3(x(1),y(1),cm_rot(3),'ko','MarkerSize',8,'LineWidth',2);
h_cm2 = plot3(x(1),y(1),cm_rot(3),'k+','MarkerSize',5,'LineWidth',2);

%Plot trajectory
h_traj = plot3(x(1),y(1),0,'b-','LineWidth',3);

%Plot trajectory - geometric center
theta_cm = atan2(CM(2),CM(1));
h_traj_center = plot3(x(1)-norm(CM(1:2))*cos(theta_cm+psi(1)),y(1)-norm(CM(1:2))*sin(theta_cm+psi(1)),0,'r-','LineWidth',3);

%Plot time
h_time = text(ws(2),ws(3),ws(6),sprintf('T_{sim}: %.2f',t(1)));

if (SHOW_FORCES==1)
    %Plot vector representing the total force
    F_tot_log_w = F_tot_log;
    for k = 1:1:length(F_tot_log(1,:))
        F_tot_log_w(:,k) = [cos(psi(k)) -sin(psi(k)); sin(psi(k)) cos(psi(k))]*F_tot_log(:,k);
    end
    size_force = 0.02;
    h_force = quiver3(H_body(1,4),H_body(2,4),H_body(3,4),size_force*F_tot_log(1,1),size_force*F_tot_log(2,1),0,'y','AutoScale','off','LineWidth',2,'MaxHeadSize',0.6);

    %Transform forces to world frame
    F1_log_w = F1_log;
    F2_log_w = F2_log;
    F3_log_w = F3_log;
    F4_log_w = F4_log;
    for k = 1:1:length(F1_log(1,:))
        F1_log_w(:,k) = [cos(psi(k)) -sin(psi(k)); sin(psi(k)) cos(psi(k))]*F1_log(:,k);
        F2_log_w(:,k) = [cos(psi(k)) -sin(psi(k)); sin(psi(k)) cos(psi(k))]*F2_log(:,k);
        F3_log_w(:,k) = [cos(psi(k)) -sin(psi(k)); sin(psi(k)) cos(psi(k))]*F3_log(:,k);
        F4_log_w(:,k) = [cos(psi(k)) -sin(psi(k)); sin(psi(k)) cos(psi(k))]*F4_log(:,k);
    end

    h_force1 = quiver3(H_w1(1,4),H_w1(2,4),H_w1(3,4),size_force*F1_log_w(1,1),size_force*F1_log_w(2,1),0,'r','AutoScale','off','LineWidth',2,'MaxHeadSize',0.6);
    h_force2 = quiver3(H_w2(1,4),H_w2(2,4),H_w2(3,4),size_force*F2_log_w(1,1),size_force*F2_log_w(2,1),0,'g','AutoScale','off','LineWidth',2,'MaxHeadSize',0.6);
    h_force3 = quiver3(H_w3(1,4),H_w3(2,4),H_w3(3,4),size_force*F3_log_w(1,1),size_force*F3_log_w(2,1),0,'b','AutoScale','off','LineWidth',2,'MaxHeadSize',0.6);
    h_force4 = quiver3(H_w4(1,4),H_w4(2,4),H_w4(3,4),size_force*F4_log_w(1,1),size_force*F4_log_w(2,1),0,'k','AutoScale','off','LineWidth',2,'MaxHeadSize',0.6);

end

hold off

pause(1.0)


%% Animation loop

k = 1;
tic

while(k<length(t))
    
    %Body
    H_body = [Rot('z',psi(k)), [x(k);y(k);r]; 0 0 0 1];
    set_block(H0*H_body*H_cm,h_body,size_body);
    
    %Wheel 1
    H_w1 = H_body*H_cm*H_w10;
    H_w1(1:3,1:3) = H_w1(1:3,1:3)*Rot('z',-wheels(1,k));
    set_cylinder(H0*H_w1,h_w1,width_wheel,r)
    
    %Wheel 2
    H_w2 = H_body*H_cm*H_w20;
    H_w2(1:3,1:3) = H_w2(1:3,1:3)*Rot('z',-wheels(2,k));
    set_cylinder(H0*H_w2,h_w2,width_wheel,r)
    
    %Wheel 3
    H_w3 = H_body*H_cm*H_w30;
    H_w3(1:3,1:3) = H_w3(1:3,1:3)*Rot('z',-wheels(3,k));
    set_cylinder(H0*H_w3,h_w3,width_wheel,r)
    
    %Wheel 4
    H_w4 = H_body*H_cm*H_w40;
    H_w4(1:3,1:3) = H_w4(1:3,1:3)*Rot('z',-wheels(4,k));
    set_cylinder(H0*H_w4,h_w4,width_wheel,r)
    
    %Trajectory
    traj_plot = [x(1:k); y(1:k); r*ones(1,k)*0; ones(1,k)];
    traj_plot = H0*traj_plot;
    set(h_traj,'XData',traj_plot(1,:),'YData',traj_plot(2,:),'ZData',traj_plot(3,:))
    
    %CM
    set(h_cm1,'XData',x(k),'YData',y(k));
    set(h_cm2,'XData',x(k),'YData',y(k));

    traj_plot = [x(1:k)-norm(CM(1:2))*cos(theta_cm+psi(1:k)); y(1:k)-norm(CM(1:2))*sin(theta_cm+psi(1:k)); r*ones(1,k)*0; ones(1,k)];
    traj_plot = H0*traj_plot;
    set(h_traj_center,'XData',traj_plot(1,:),'YData',traj_plot(2,:),'ZData',traj_plot(3,:))
    
    %Time
    h_time.String = sprintf('T_{sim}: %.2f',t(k));
    
    if (SHOW_FORCES == 1)
        %Force
        H_body_in_cm = H_body*H_cm;
        set(h_force,'XData',H_body(1,4),'YData',H_body(2,4),'UData',size_force*F_tot_log_w(1,k),'VData',size_force*F_tot_log_w(2,k));

        set(h_force1,'XData',H_w1(1,4),'YData',H_w1(2,4),'UData',size_force*F1_log_w(1,k),'VData',size_force*F1_log_w(2,k));
        set(h_force2,'XData',H_w2(1,4),'YData',H_w2(2,4),'UData',size_force*F2_log_w(1,k),'VData',size_force*F2_log_w(2,k));
        set(h_force3,'XData',H_w3(1,4),'YData',H_w3(2,4),'UData',size_force*F3_log_w(1,k),'VData',size_force*F3_log_w(2,k));
        set(h_force4,'XData',H_w4(1,4),'YData',H_w4(2,4),'UData',size_force*F4_log_w(1,k),'VData',size_force*F4_log_w(2,k));
    end
    
    drawnow

    %Control simulation time
    k = find(t>toc*SPEED,1);
    
end

