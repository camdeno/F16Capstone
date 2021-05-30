%% System identification of a small RC F16 model
% Data source: RelFlight 9.5 simulator, F16 ducted fan model

%% Longitudial model
%
% References:
% 1) Learjet paper: 
% https://github.com/camdeno/F16Capstone/tree/main/Literature#development-and-validation-of-a-flight-identified-full-envelope-business-jet-simulation-model-using-a-stitching-architecture
% pages 11 and 12
% 2) SysID book:
% https://github.com/camdeno/F16Capstone/tree/main/Literature#aircraft-and-rotorcraft-system-identification
% pages Chapter 13
%
%
% Data: 25_05_2021_23_20_29.mat (recorded response in steady state flight
% to elevator input

clear all;
clc;

%% Longidutial model
%Alon = [[Xu, Xw, Xq-W0, -g*cos(Theta0)];...
%        [Zu, Zw, Zq+U0, -g*sin(Theta0)];... 
%        [Mu, Mw, Mq, 0]; 
%        [0, 0, 1, 0]];
% x = [u w q theta]
% Filled with Learjet values as a start
A =    [   -0.0084    0.0862         0   32.1595;
           -0.1241   -1.4300         0    0.9651;
            0.0002   -0.0235   -1.6520         0;
                 0         0    1.0000         0];

%Blon = [[Xde, Xdt];... 
%        [Zde, Zdt];... 
%        [Mde, Mdt];... 
%        [0, 0]];
% u = [delta_elevator, delta_throttle]
B = [   0.0714    0.0023;
        -1.2490  -0.0011;
        -0.1920  -0.0000;
              0        0];
          

% system dimension parameters
n_states = size(A);
n_states = n_states(1);
n_inputs = size(B);
n_inputs = n_inputs(2);

% initial system
init_sys = idss(A, B,...                                             % create id ss system
    eye(n_states), zeros(n_states, n_inputs),...                           % canonical form w/ no throughput
    zeros(n_states, n_states), ...                                         % error
    [0.0, 0.0, 0.0, 0.0], 0);                          % initial state and continuous system


%% System ID Constraints
% fix system values (enforce separable lat /lon models)
init_sys.Structure.A.Free = (A~=0 & A~=1.0);                               % fix zeros and ones
init_sys.Structure.B.Free = (B~=0 & B~=1.0);                               % fix zeros and ones
init_sys.Structure.C.Free = false;                                         % don't solve
init_sys.Structure.D.Free = false;                                         % don't solve

%% Training Data Creation
% Tasks
% 1) use your script to select *two* sets of data - the requerements are
% - starts as constant steady flight
 first = load('Elev_Clipped_1.mat');
 second = load('Elev_Clipped_2.mat');

% - as little lateral move as possible?
% - elevator input contains a frequency sweep
% 2) The variables we need (for both lat and lon modes):
% - timestamp (to calculate delta T)
time_1 = first.vals(:,38);
ts_1 = mean(diff(time_1));
time_2 = second.vals(:,38);
ts_2 = mean(diff(time_2));
% - body velocity U
bodyVel_U1 = first.vals(:,16);
bodyVel_U2 = second.vals(:,16);
% - body velocuty V
bodyVel_V1 = first.vals(:,17);
bodyVel_V2 = second.vals(:,17);
% - body velocity W
bodyVel_W1 = first.vals(:,18);
bodyVel_W2 = second.vals(:,18);
% - roll rate p (in rad/s!)
rollRate_1 = first.vals(:,6).*(pi/180);
rollRate_2 = second.vals(:,6).*(pi/180);
% - pitch rate q (in rad/s!)
pitchRate_1 = first.vals(:,5).*(pi/180);
pitchRate_2 = second.vals(:,5).*(pi/180);
% - yaw rate r (in rad/s!)
yawRate_1 = first.vals(:,7).*(pi/180);
yawRate_2 = second.vals(:,7).*(pi/180);
% - roll angle phi (in rads!) 
rollAng_1 = first.vals(:,10).*(pi/180);
rollAng_2 = second.vals(:,10).*(pi/180);
% - pitch angle theta (in rads!) - it is called "inclination" in the CSV data
pitchAng_1 = first.vals(:,9).*(pi/180);
pitchAng_2 = second.vals(:,9).*(pi/180);
% - yaw angle psi (in rads!) - it is called "azimuth" in the CSV data
yawAng_1 = first.vals(:,8).*(pi/180);
yawAng_2 = second.vals(:,8).*(pi/180);
% - elevator input
elevIn_1 = first.vals(:,45);
elevIn_2 = second.vals(:,45);
% - throttle input
throttleIn_1 = first.vals(:,46);
throttleIn_2 = second.vals(:,46);
% - aileron input
ailIn_1 = first.vals(:,44);
ailIn_2 = second.vals(:,44);
% - rudder input
rudIn_1 = first.vals(:,47);
rudIn_2 = second.vals(:,47);
% 3) Because the model calculates only a difference from some "trimmed"
% input/state we need to find out what are the trimmed values for all
% variables denoted above - you can mark them with "_0"
% The data you use then have this trimmed value subtracted 
% 3) put data into a data structure
% data = iddata(x, u, ts);                                                   % put trajectories into id data structure
% where ts is the average sampling period
% 4) the same for the validation data (second set)
%% System Identification
% data in this case are the training data (first set)
lsys = ssest(data, init_sys, 'DisturbanceModel','none');                   % estimate the linear system dynamics


%% Latereal mode
% lateral model
% Alat = [[Yv, Yp+W0, Yr-U0, g*cos(Theta0)];...
%         [Lv, Lp, Lr, 0];...
%         [Nv, Np, Nr, 0];...
%         [0, 1, tan(Theta0), 0]];
% x = [v p r phi]
% Blat = [[Yda, Ydr];...
%        [Lda, Ldr];...
%        [Nda, Ndr];...
%        [0, 0]];
% u = [delta_aileron, delta_rudder]

%% Both modes at the same time
% Perhaps there is enough cross-coupling to do both in one shot
%{
A = (sym 8×8 matrix)
  ⎡Xu  Xw  -W₀ + Xq  -g⋅cos(Θ₀)  0      0        0          0    ⎤
  ⎢                                                              ⎥
  ⎢Zu  Zw  U₀ + Zq   -g⋅sin(Θ₀)  0      0        0          0    ⎥
  ⎢                                                              ⎥
  ⎢Μ   Mw     Mq         0       0      0        0          0    ⎥
  ⎢                                                              ⎥
  ⎢0   0      1          0       0      0        0          0    ⎥
  ⎢                                                              ⎥
  ⎢0   0      0          0       Yv  W₀ + Yp  -U₀ + Yr  g⋅cos(Θ₀)⎥
  ⎢                                                              ⎥
  ⎢0   0      0          0       Lv    Lp        Lr         0    ⎥
  ⎢                                                              ⎥
  ⎢0   0      0          0       Nv    Np        Nr         0    ⎥
  ⎢                                                              ⎥
  ⎣0   0      0          0       0      1     tan(Θ₀)       0    ⎦
B = (sym 8×4 matrix)
  ⎡Xde  Xdt   0    0 ⎤
  ⎢                  ⎥
  ⎢Zde  Zdt   0    0 ⎥
  ⎢                  ⎥
  ⎢Mde  Mdt   0    0 ⎥
  ⎢                  ⎥
  ⎢ 0    0    0    0 ⎥
  ⎢                  ⎥
  ⎢ 0    0   Yda  Ydr⎥
  ⎢                  ⎥
  ⎢ 0    0   Lda  Ldr⎥
  ⎢                  ⎥
  ⎢ 0    0   Nda  Ndr⎥
  ⎢                  ⎥
  ⎣ 0    0    0    0 ⎦
 %}

A =    [   -0.0084    0.0862         0   32.1595         0         0         0         0;
           -0.1241   -1.4300         0    0.9651         0         0         0         0;
            0.0002   -0.0235   -1.6520         0         0         0         0         0;
                 0         0    1.0000         0         0         0         0         0;
                 0         0         0         0   -0.1698    0.8673         0  -32.1595;
                 0         0         0         0   -0.0192   -2.2780    0.8487         0;
                 0         0         0         0    0.0053   -0.2258   -0.2719         0;
                 0         0         0         0         0    1.0000    0.0300         0];

B = [   0.0714   0.0023        0        0;
        -1.2490  -0.0011        0        0;
        -0.1920  -0.0000        0        0;
              0        0        0        0;
              0        0  -0.0132   0.3073;
              0        0  -0.1623   0.0330;
              0        0  -0.0113  -0.0373;
              0        0        0        0];

% 