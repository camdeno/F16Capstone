% Cam Osborn 2/28/2020
% F16TestCase Declares the control vector and the state vector.
% The script then calls the F16sixDegreeFreedom_SC to test the inputs
% with the state derivative vector. 
% INPUTS:
%    u(1) = Throttle
%    u(2) = Elevator
%    u(3) = Aileron
%    u(4) = Rudder
% OUTPUTS:
%    x(1)  = Airspeed
%    x(2)  = Alpha
%    x(3)  = Beta
%    x(4)  = Phi
%    x(5)  = Theta
%    x(6)  = Psi
%    x(7)  = P
%    x(8)  = Q
%    x(9)  = R
%    x(12) = Altitude
%    x(13) = Power

% Declare Input Vector
u = [0.9;20;-15;-20;];

% Declare State Vector
x = [70; 0.5; -0.2; -1; 1; -1; 0.7; -0.8; 0.9; 0; 0; 10000; 90;];

% Call F-16 6DoF File for to compare outputs with Table 3.5-2
F16sixDegreeFreedom_SC(x,u)