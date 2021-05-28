%% Learjet System ID Example
% Author: Ethan Lew <elew@galois.com>
% Date: 05/27/2021
%
% Demonstrates the usage of the MATLAB system ID toolbox to recover linear
% learjet dynamics (learjet.m). We use constraints to limit the system
% solutions and we are able to recover the correct dynamics with aircraft
% state trajectories with random inputs applied.
%
%% Learjet Model Definition
%{
I don't have the Matlab symbolic toolbox, so I pulled some values from my
octave script learjet.m. The equations are:

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

% system dimension parameters
n_states = size(A);
n_states = n_states(1);
n_inputs = size(B);
n_inputs = n_inputs(2);

% initial system
sysp = ss(A, B, eye(n_states), zeros(n_states, n_inputs), 0);              % create ss plant

%% Learjet Model Control
% For this exercise, we will be designing a LQR model to control the
% unstable learjet plant. This is required for the ssest system id
% function, which requires a stable, linear system.

[K,S,e] = lqr(A, B, 10.0, 0.1, 0);                                         % LQR control policy
sysc = ss((A-B*K), B, eye(n_states), zeros(n_states, n_inputs));           % create ss controlled system

init_sys = idss((A-B*K), B,...                                             % create id ss controlled system
    eye(n_states), zeros(n_states, n_inputs),...                           % canonical form w/ no throughput
    zeros(n_states, n_states), ...                                         % error
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0);                          % initial state and continuous system

figure
pzmap(sysp, sysc)                                                          % pole zero map of plant vs controlled system
legend("Plant System", "Controlled System")
grid on

%% System ID Constraints
% fix system values (enforce separable lat /lon models)
init_sys.Structure.A.Free = (A~=0 & A~=1.0);                               % fix zeros and ones
init_sys.Structure.B.Free = (B~=0 & B~=1.0);                               % fix zeros and ones
init_sys.Structure.C.Free = false;                                         % don't solve
init_sys.Structure.D.Free = false;                                         % don't solve

%% Training Data Creation
ts = 0.01;                                                                 % sampling period
tspan = [0, 40];                                                           % training traces timespan
t = linspace(tspan(1), tspan(2) - ts, (tspan(2)- tspan(1))/ts);            % time points to evaluate
u = randn((tspan(2)- tspan(1))/ts, 4);                                     % uncorrelated noise
u = smoothdata(u, 'gaussian', 100);                                        % not necessary, but add some correlation
y = lsim(init_sys, u, t, [0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);          % state time traces
data = iddata(y, u, ts);                                                   % put trajectories into id data structure

%% System Identification
lsys = ssest(data, init_sys, 'DisturbanceModel','none');                   % estimate the linear system dynamics
lsysp = ss(lsys.A + lsys.B * K, lsys.B, lsys.C, lsys.D);                   % infer plant from the system estimate
yt = lsim(lsys, u, t, [0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);             % get time traces for new system

figure
step(sysc, lsys)                                                           % step response
legend("True Controlled System", "Estimated Controlled System")
