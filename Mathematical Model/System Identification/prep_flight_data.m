%% Convert to Mat
clear all
clc

vals = readtable('./25_05_2021__23_20_29.csv');
save('./25_05_2021__23_20_29.mat','vals')
%% Load data
clear all
clc
load('./25_05_2021__23_20_29.mat')
% vals.Properties.VariableDescriptions has the header

%% Longitudial model
% Stevens:  x=[alpha, q, v_t, phi] u = [delta_e, delta_t]
VT_MPS_IDX = 1;
INCLINATION_DEG_IDX = 9; % NOTE: we don't have AoA directly...
Q_DEGS_IDX = 5;
% Gotta get quats
QUAT_W_IDX = 43;
QUAT_X_IDX = 40;
QUAT_Y_IDX = 41;
QUAT_Z_IDX = 42;

%% Lateral model
% Stevens: x= [beta, roll, roll_rate, yaw_rate] u = [delta_aileron, delta_rudder]
ROLL_DEG_IDX = 10;
ROLL_RATE_DEG_S_IDX = 6;
YAW_RATE_DEG_S_IDX = 7;

%% Common
% Time
TIME_SEC_IDX = 38;
% Inputs
RC_0_IDX = 44;
RC_1_IDX = 45;
RC_2_IDX = 46;
RC_3_IDX = 47;
RC_4_IDX = 48;
RC_5_IDX = 49;
RC_6_IDX = 50;
RC_7_IDX = 51;

% Get variables
vt_full = table2array(vals(:,VT_MPS_IDX));
incl_full = table2array(vals(:, INCLINATION_DEG_IDX));
q_full = table2array(vals(:,Q_DEGS_IDX));
quat_w = table2array(vals(:,QUAT_W_IDX));
quat_x = table2array(vals(:,QUAT_X_IDX));
quat_y = table2array(vals(:,QUAT_Y_IDX));
quat_z = table2array(vals(:,QUAT_Z_IDX));
t_full = table2array(vals(:,TIME_SEC_IDX));
rc_0 = table2array(vals(:,RC_0_IDX)); % roll
rc_1 = table2array(vals(:,RC_1_IDX)); % pitch
rc_2 = table2array(vals(:,RC_2_IDX)); % throttle
rc_3 = table2array(vals(:,RC_3_IDX)); % yaw
rc_4 = table2array(vals(:,RC_4_IDX)); % Null
rc_5 = table2array(vals(:,RC_5_IDX)); % Null
rc_6 = table2array(vals(:,RC_6_IDX)); % gear
rc_7 = table2array(vals(:,RC_7_IDX)); % Null
%%
plot(t_full, rc_7)