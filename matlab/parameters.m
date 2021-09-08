

states = [0.00; 0.00; 0.0; 0.0; 0.0; 0.0]; %initial states
wheels = [0; 0; 0; 0]; %initial orientation of the wheels (only for plot)

g = 9.81; %gravity acceleration


% Select the robot type
robot = 'EspeleoRobo';
% robot = 'PioneerP3AT';


if strcmp(robot,'EspeleoRobo')
    %Espeleorobo
    a = 0.215; %foward/backward distance of the weels (from the robot's center)
    b = 0.18; %lateral distance of the weels (from the robot's center)
    r = 0.151; %radius of the wheels
    epsilon = 0.005; %"velocity of the maximum static friction"
    m = 27.4; %mass
    J = 0.76; %moment of inertia
    kf = 0.48; %coefficient of the kinetic friction (N/N)
    %Center of mass with respect to the body's center (x, y) and floor 
    CM = [0.0; 0.0; 0.12];
    
    %Size of the robot (x, y, z) - only for animation
    size_body = [0.55 0.35 0.12];
    
elseif('PioneerP3AT')
    %Pioneer
    a = 0.135; %foward/backward distance of the weels (from the robot's center)
    b = 0.2; %lateral distance of the weels (from the robot's center)
    r = 0.098; %radius of the wheels
    epsilon = 0.005; %"velocity of the maximum static friction"
    J = 0.58; %moment of inertia
    m = 26.8; %mass
    kf = 0.42; %experimental; %coefficient of the kinetic friction (N)
    %Center of mass with respect to the body's center (x, y) and floor 
    CM = [0.0; 0.0; 0.15];
    
    %Size of the robot (x, y, z) - only for animation
    size_body = [0.4 0.34 0.18];
    
else
    error('Select the robot')
end


%Compute the vectors that go from the robot's center of mass to the wheels
c1 = [a; -b]-CM(1:2);
c2 = [-a; -b]-CM(1:2);
c3 = [-a; b]-CM(1:2);
c4 = [a; b]-CM(1:2);

%Simulaion times definitions
T = 10;
dt = 0.001;
t = 0:dt:T;


%Binary flag to plot simulated data
DO_PLOTS = 0;

%Flag to run an animation
% 0 - no animation
% 1 - 3D animation
% 2 - 2D animation
RUN_ANIMATION = 1;

%Simulation speed factor
SPEED = 4.0;
