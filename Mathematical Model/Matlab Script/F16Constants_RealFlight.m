% Initialize Constants for F16 Simulation
clear
clc
 load('Control_Inputs.mat')
 load('time');
 time = time-time(1);

 simin= timeseries(control_IN,time);
%% Define Constants

% x - State Vector

%     VT          = x(1);          % Freestream Airspeed
%     Alpha       = x(2)*RTOD;     % Angle of Attack normalized to Degrees
%     Beta        = x(3)*RTOD;     % Angle of Sideslip normalized to Degrees
%     Phi         = x(4);          % Euler Angle - X Component (Bank)
%     Theta       = x(5);          % Euler Angle - Y Component (Pitch)
%     Psi         = x(6);          % Euler Angle - Z Component (Yaw)
%     P           = x(7);          % Angular Velocity - X Component
%     Q           = x(8);          % Angular Velocity - Y Component
%     R           = x(9);          % Angular Velocity - Z Component
%     Pn          = x(10);         % North Displacement
%     Pe          = x(11);         % East Displacement
%     alt         = x(12);         % Altitude 
%     POW         = x(13);         % Engine Power State 
% 
%     Thtl        = u(1);          % Throttle
%     Elev        = u(2);          % Elevator
%     Ail         = u(3);          % Aileron
%     Rdr         = u(4);          % Rudder

% initial conditions
x0 = [24; 0;0; 0; 0;0; 0; 0; 0; 0; 0; 500; 100;];
%x0 = [500; 0.038587835162713;  0;  0;  0.038587835162713;0; 0; 0; 0; 0; 0; 300; 0.008983182660417;];

% u - Control Vector
%u = [ control_IN(:,1); control_IN(:,2); control_IN(:,3);  control_IN(:,4);];

% Simulation Time
TF = time;
%% Run Simulation
sim('F16Simulation_SC_RealFlight.slx')
%% Plot Results
t = out.simout.Time;

T=t;
x1  = out.simout.Data(:,1); 
x2  = out.simout.Data(:,2); 
x3  = out.simout.Data(:,3);
x4  = out.simout.Data(:,4); 
x5  = out.simout.Data(:,5); 
x6  = out.simout.Data(:,6); 
x7  = out.simout.Data(:,7); 
x8  = out.simout.Data(:,8); 
x9  = out.simout.Data(:,9); 
x10 = out.simout.Data(:,10); 
x11 = out.simout.Data(:,11); 
x12 = out.simout.Data(:,12); 
x13 = out.simout.Data(:,13);

u1  = out.U.Data(:,1);
u2  = out.U.Data(:,2);
u3  = out.U.Data(:,3);
u4  = out.U.Data(:,4);

%% Plot State Variable Results
figure;

subplot(5,3,1)
plot(t,x1)
grid on
title('Airspeed')
xlabel('Time (sec)')
ylabel ('Velocity (ft/s)')

subplot(5,3,2)
plot(t,x2)
grid on
title('Alpha')
xlabel('Time (sec)')
ylabel('Degrees')

subplot(5,3,3)
plot(t,x3)
%ylim([-2 2])
grid on
title('Beta')
xlabel('Time (sec)')
ylabel('Degrees')

subplot(5,3,4)
plot(t,x4)
grid on
%ylim ([-4 8])
title('Phi')
xlabel('Time (sec)')
ylabel('rad')

subplot(5,3,5)
plot(t,x5)
grid on
title('Theta')
%ylim ([-2 2])
xlabel('Time (sec)')
ylabel('rad')

subplot(5,3,6)
plot(t,x6)
%ylim([-4 8])
grid on
title('Psi')
xlabel('Time (sec)')
ylabel('rad')

subplot(5,3,7)
plot(t,x7)
%ylim ([-2 2])
grid on
title('P')
ylabel('rad/s')
xlabel('Time (sec)')

subplot(5,3,8)
plot(t,x8)
%ylim ([-2 2])
grid on
title('Q')
xlabel('Time (sec)')
ylabel('rad/s')

subplot(5,3,9)
plot(t,x9)
%ylim ([-2 2])
grid on
title('R')
ylabel('rad/s')
xlabel('Time (sec)')

subplot(5,3,10)
plot(t,x10)
grid on
title('North Displacement')
xlabel('Time (sec)')

subplot(5,3,11)
plot(t,x11)
grid on
title('East Displacement')
xlabel('Time (sec)')


subplot(5,3,12)
plot(t,x12)
%ylim([0 2000])
grid on
title('Altitude')
xlabel('Time (sec)')
ylabel('Height (ft)')

subplot(5,3,13)
plot(t,x13)
grid on
title('Engine Power')
xlabel('Time (sec)')

subplot(5,3,14)
plot(x10,x11)
grid on
title('Flight Path')
xlabel('North')
ylabel('East')

% figure;
% title('3D Plot of Position')
% plot(x11,x12,x13)
% xlabel('North')
% ylabel('East')
% zlabel('Altitude')

%% Plot Control Surfaces
figure;

subplot(4,1,1)
plot(u1)

grid on
title('Throttle Setting')
xlabel('Time (sec)')

subplot(4,1,2)
plot(u2)
grid on
title('Elevator Setting')
xlabel('Time (sec)')
ylabel('Degree')

subplot(4,1,3)
plot(u3)

grid on
title('Aileron Setting')
xlabel('Time (sec)')
ylabel('Degree')

subplot(4,1,4)
plot(u4)

grid on
title('Rudder')
xlabel('Time (sec)')
ylabel('Degree')



