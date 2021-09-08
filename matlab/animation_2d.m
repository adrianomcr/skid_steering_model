

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
plot(0,0,'k')
xlabel('x')
ylabel('y')
axis equal
axis(ws(1:4))
grid on

hold on
%Plot floor
H_floor = [Rot('z',pi/2)*Rot('y',pi/2), [0;0;0]; 0 0 0 1];
plot(ws([2 1 1 2 2]),ws([3 3 4 4 3]),'k')

%Plot body
H_body = [Rot('z',psi(1)), [x(1);y(1);r]; 0 0 0 1];
body_face0 = [[1 -1 -1 1]*0.55/2.0; [-1 -1 1 1]*0.30/2.0; [0 0 0 0]; [1 1 1 1]];
body_face = H0*H_body*H_cm*body_face0;
h_body = fill(body_face(1,:),body_face(2,:),'k','FaceColor',[0 0 0]+0.5);%,'FaceAlpha',0.8);


wheel_base = [[1 -1 -1 1]*r;[-1 -1 1 1]*0.02;[0 0 0 0];[1 1 1 1]];
H_w10 = eye(4); H_w10(1:2,4) = [a; -b];
H_w20 = eye(4); H_w20(1:2,4) = [-a; -b];
H_w30 = eye(4); H_w30(1:2,4) = [-a; b];
H_w40 = eye(4); H_w40(1:2,4) = [a; b];

drawnow
width_wheel = 0.04;
number_faces = 10;

%Plot wheel 1
H_w1 = H0*H_body*H_w10*H_cm;
wheel1 = H_w1*wheel_base;
h_w1 = fill(wheel1(1,:),wheel1(2,:),'r');
%Plot wheel 2
H_w2 = H0*H_body*H_w20*H_cm;
wheel2 = H_w2*wheel_base;
h_w2 = fill(wheel2(1,:),wheel2(2,:),'c');

%Plot wheel 3
H_w3 = H0*H_body*H_w30*H_cm;
wheel3 = H_w3*wheel_base;
h_w3 = fill(wheel3(1,:),wheel3(2,:),'c');

%Plot wheel 4
H_w4 = H0*H_body*H_w40*H_cm;
wheel4 = H_w4*wheel_base;
h_w4 = fill(wheel4(1,:),wheel4(2,:),'r');


%%

%Plot CM
h_cm1 = plot(x(1),y(1),'ko','MarkerSize',8,'LineWidth',2);
h_cm2 = plot(x(1),y(1),'k+','MarkerSize',5,'LineWidth',2);

%Plot trajectory
%h_traj = plot3(x(1),y(1),0,'b-','LineWidth',3);
h_traj = plot(x(1),y(1),'b-','LineWidth',3);

%Plot time
%h_time = text(ws(2),ws(3),ws(6),sprintf('T_{sim}: %.2f',t(1)));
h_time = text(ws(2),ws(3),sprintf('T_{sim}: %.2f',t(1)));


if (SHOW_FORCES==1)
  %Plot vector representing the total force
  F_tot_log_w = F_tot_log;
  for k = 1:1:length(F_tot_log(1,:))
      F_tot_log_w(:,k) = [cos(psi(k)) -sin(psi(k)); sin(psi(k)) cos(psi(k))]*F_tot_log(:,k);
  end
  size_force = 0.02;
  %h_force = quiver3(H_body(1,4),H_body(2,4),H_body(3,4),size_force*F_tot_log(1,1),size_force*F_tot_log(2,1),0,'y','AutoScale','off','LineWidth',2,'MaxHeadSize',0.6);
  h_force = quiver(H_body(1,4),H_body(2,4),H_body(3,4),size_force*F_tot_log(1,1),size_force*F_tot_log(2,1),'y','AutoScale','off','LineWidth',2,'MaxHeadSize',0.06);

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

  h_force1 = quiver(H_w1(1,4),H_w1(2,4),H_w1(3,4),size_force*F1_log_w(1,1),size_force*F1_log_w(2,1),'r','AutoScale','off','LineWidth',2,'MaxHeadSize',0.06);
  h_force2 = quiver(H_w2(1,4),H_w2(2,4),H_w2(3,4),size_force*F2_log_w(1,1),size_force*F2_log_w(2,1),'g','AutoScale','off','LineWidth',2,'MaxHeadSize',0.06);
  h_force3 = quiver(H_w3(1,4),H_w3(2,4),H_w3(3,4),size_force*F3_log_w(1,1),size_force*F3_log_w(2,1),'b','AutoScale','off','LineWidth',2,'MaxHeadSize',0.06);
  h_force4 = quiver(H_w4(1,4),H_w4(2,4),H_w4(3,4),size_force*F4_log_w(1,1),size_force*F4_log_w(2,1),'k','AutoScale','off','LineWidth',2,'MaxHeadSize',0.06);

end


hold off

pause(1.0)


%% Animation loop


k = 1;
tic

while(k<length(t))
    
    %Body
    H_body = [Rot('z',psi(k)), [x(k);y(k);r]; 0 0 0 1];
    body_face = H0*H_body*H_cm*body_face0;
    set(h_body,'XData',body_face(1,:),'YData',body_face(2,:));
    
    %Wheel 1
    H_w1 = H0*H_body*H_w10*H_cm;
    wheel1 = H_w1*wheel_base;
    set(h_w1,'XData',wheel1(1,:),'YData',wheel1(2,:));
    
    %Wheel 2
    H_w2 = H0*H_body*H_w20*H_cm;
    wheel2 = H_w2*wheel_base;
    set(h_w2,'XData',wheel2(1,:),'YData',wheel2(2,:));
    
    %Wheel 3
    H_w3 = H0*H_body*H_w30*H_cm;
    wheel3 = H_w3*wheel_base;
    set(h_w3,'XData',wheel3(1,:),'YData',wheel3(2,:));
    
    %Wheel 4
    H_w4 = H0*H_body*H_w40*H_cm;
    wheel4 = H_w4*wheel_base;
    set(h_w4,'XData',wheel4(1,:),'YData',wheel4(2,:));
    
    %Trajectory
    traj_plot = [x(1:k); y(1:k); r*ones(1,k)*0; ones(1,k)];
    traj_plot = H0*traj_plot;
    set(h_traj,'XData',traj_plot(1,:),'YData',traj_plot(2,:))
    
    %CM
    set(h_cm1,'XData',x(k),'YData',y(k));
    set(h_cm2,'XData',x(k),'YData',y(k));
    
    %Time
    set(h_time,'String',sprintf('T_{sim}: %.2f',t(k)));
   
  
    if (SHOW_FORCES == 1)
      H_body_in_cm = H_body*H_cm;
      set(h_force,'XData',H_body_in_cm(1,4),'YData',H_body_in_cm(2,4),'UData',size_force*F_tot_log_w(1,k),'VData',size_force*F_tot_log_w(2,k));
      
      set(h_force1,'XData',H_w1(1,4),'YData',H_w1(2,4),'UData',size_force*F1_log_w(1,k),'VData',size_force*F1_log_w(2,k));
      set(h_force2,'XData',H_w2(1,4),'YData',H_w2(2,4),'UData',size_force*F2_log_w(1,k),'VData',size_force*F2_log_w(2,k));
      set(h_force3,'XData',H_w3(1,4),'YData',H_w3(2,4),'UData',size_force*F3_log_w(1,k),'VData',size_force*F3_log_w(2,k));
      set(h_force4,'XData',H_w4(1,4),'YData',H_w4(2,4),'UData',size_force*F4_log_w(1,k),'VData',size_force*F4_log_w(2,k));
    end
      
    drawnow

    %Control simulation time
    k = find(t>toc*SPEED,1);
    
end

