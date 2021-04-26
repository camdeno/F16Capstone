% Initialize Constants for F16 Simulation
clear
clc

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
%                 = x(10);         % North Displacement
%                 = x(11);         % East Displacement
%     alt         = x(12);         % Altitude 
%     POW         = x(13);         % Engine Power State 
% 
%     Thtl        = u(1);          % Throttle
%     Elev        = u(2);          % Elevator
%     Ail         = u(3);          % Aileron
%     Rdr         = u(4);          % Rudder

% initial conditions
%x0 = [100; 0;0; 0; 0;0; 0; 0; 0; 0; 0; 1000; 0;];
x0 = [500; 0;0; 0; 0;0; 0; 0; 0; 0; 0; 1000; 0;];

% u - Control Vector
u = [ 0; 20; 20; 0;];

% Simulation Time
TF = 30;
%% Run Simulation
    % To simulate Small Scale replace sim file with F16Simulation_SC
    % To simulate Large Scale replace sim file with Testsimulation
disp('Do you want to run the Full Scale model or the Small Scale Model?' )
disp('Input 1 to run the Small Scale or Input 2 to run the Large Scale: ')
x = input(' ');
if (x == 1)
    sim('F16Simulation_SC.slx')
elseif (x == 2)
    sim('Testsimulation.slx')
end 
%% Plot Results
t = ans.simout.Time;

T=t;
x1 = ans.simout.Data(:,1); 
x2 = ans.simout.Data(:,2); 
x3 = ans.simout.Data(:,3);
x4 = ans.simout.Data(:,4); 
x5 = ans.simout.Data(:,5); 
x6 = ans.simout.Data(:,6); 
x7 = ans.simout.Data(:,7); 
x8 = ans.simout.Data(:,8); 
x9 = ans.simout.Data(:,9); 
x10 = ans.simout.Data(:,10); 
x11 = ans.simout.Data(:,11); 
x12 = ans.simout.Data(:,12); 
x13 = ans.simout.Data(:,13);

u1 = ans.U.Data(:,1);
u2 = ans.U.Data(:,2);
u3 = ans.U.Data(:,3);
u4 = ans.U.Data(:,4);

%% Plot State Variable Results
figure;
if (x==1)
    sgtitle('State Variables Small Scale')
elseif (x==2)
    sgtitle('State Variables Large Scale')
end

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
if (x==1)
sgtitle('Control Inputs Small Scale')
elseif (x==2)
sgtitle('Control Inputs Large Scale')
end

subplot(4,1,1)
plot(t,u1)

grid on
title('Throttle Setting')
xlabel('Time (sec)')

subplot(4,1,2)
plot(t,u2)
grid on
title('Elevator Setting')
xlabel('Time (sec)')
ylabel('Degree')

subplot(4,1,3)
plot(t,u3)

grid on
title('Aileron Setting')
xlabel('Time (sec)')
ylabel('Degree')

subplot(4,1,4)
plot(t,u4)

grid on
title('Rudder')
xlabel('Time (sec)')
ylabel('Degree')



