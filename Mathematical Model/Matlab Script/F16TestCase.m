% Cam Osborn 2/28/2020
% F16TestCase Declares the control vector and the state vector.
% The script then calls the F16sixDegreeFreedom to test the inputs
% with the state derivative vector. 
% INPUTS:
%    
% OUTPUTS:
%   

% Declare Input Vector
u = [0.9;20;-15;-20;];

% Declare State Vector
x = [500; 0.5; -0.2; -1; 1; -1; 0.7; -0.8; 0.9; 1000; 900; 10000; 90;];

% Call F-16 6DoF File for to compare outputs with Table 3.5-2
F16sixDegreeFreedom(x,u)