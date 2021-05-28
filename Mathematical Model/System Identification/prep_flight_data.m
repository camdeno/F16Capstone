clc;
clear
format long
%% Get User Input
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Kesh Ikuma (2021). inputsdlg: Enhanced Input Dialog Box 
% (https://www.mathworks.com/matlabcentral/fileexchange/25862-inputsdlg-enhanced-input-dialog-box), 
% MATLAB Central File Exchange. Retrieved May 27, 2021.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

prompt = {'Enter the file name:';'Enter the file type:'};
name = 'Input for data handeling';

formats = struct('type', {}, 'style', {}, 'items', {}, ...
  'format', {}, 'limits', {}, 'size', {});
formats(1,1).type   = 'edit';
formats(1,1).format = 'text';
formats(1,1).limits = [1 1];

formats(2,1).type   = 'list';
formats(2,1).style  = 'popupmenu';
formats(2,1).items  = {'CSV', 'MAT', 'ULG'};
defaultanswer = {'./25_05_2021__23_20_29.mat', 2};
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[answer, canceled] = inputsdlg(prompt, name, formats, defaultanswer);
if canceled == 1
    disp("Canceled, please try again")
elseif answer{2,1} == 2 
    load(answer{1,1});    
elseif answer{2,1} == 1
    vals = readmatrix(answer{1,1});
    save('file.mat', 'vals'); 
    load('file.mat');
    vals = array2table(vals(:,:)); 
end
%% Add a header to the table
header = ["m_airspeed_MPS","m_altitudeASL_MTR","m_altitudeAGL_MTR","m_groundspeed_MPS" ...
    "m_pitchRate_DEGpSEC","m_rollRate_DEGpSEC","m_yawRate_DEGpSEC","m_azimuth_DEG","m_inclination_DEG"...
    "m_roll_DEG","m_aircraftPositionX_MTR","m_aircraftPositionY_MTR","m_velocityWorldU_MPS"...
    "m_velocityWorldV_MPS","m_velocityWorldW_MPS","m_velocityBodyU_MPS","m_velocityBodyV_MPS"...
    "m_velocityBodyW_MPS","m_accelerationWorldAX_MPS2","m_accelerationWorldAY_MPS2","m_accelerationWorldAZ_MPS2"...
    "m_accelerationBodyAX_MPS2","m_accelerationBodyAY_MPS2","m_accelerationBodyAZ_MPS2","m_windX_MPS","m_windY_MPS"...
    "m_windZ_MPS","m_propRPM","m_heliMainRotorRPM","m_batteryVoltage_VOLTS","m_batteryCurrentDraw_AMPS"...
    "m_batteryRemainingCapacity_MAH","m_fuelRemaining_OZ","m_isLocked","m_hasLostComponents"...
    "m_anEngineIsRunning","m_isTouchingGround","m_currentPhysicsTime_SEC","m_currentPhysicsSpeedMultiplier"...
    "m_orientationQuaternion_X","m_orientationQuaternion_Y","m_orientationQuaternion_Z","m_orientationQuaternion_W"...
    "rc_channel_0","rc_channel_1","rc_channel_2","rc_channel_3","rc_channel_4","rc_channel_5","rc_channel_6","rc_channel_7"...
    "rc_channel_8","rc_channel_9","rc_channel_10","rc_channel_11"];

for idx = 1:size(header,2);
    expr = sprintf("%s = %d",header(idx),idx);
    eval(expr);
end
%% Prepare Data to be moved from table to array 

Data_types = {'True', 'False'};
Data_value = [      1,        0];
[wasfound, idx] = ismember(vals.m_isLocked, Data_types);
f_values = nan(length(idx), 1);
f_values(wasfound) = Data_value(idx(wasfound));
vals.m_isLocked = f_values;

[wasfound, idx] = ismember(vals.m_hasLostComponents, Data_types);
f_values = nan(length(idx), 1);
f_values(wasfound) = Data_value(idx(wasfound));
vals.m_hasLostComponents = f_values;

[wasfound, idx] = ismember(vals.m_anEngineIsRunning, Data_types);
f_values = nan(length(idx), 1);
f_values(wasfound) = Data_value(idx(wasfound));
vals.m_anEngineIsRunning = f_values;

[wasfound, idx] = ismember(vals.m_isTouchingGround, Data_types);
f_values = nan(length(idx), 1);
f_values(wasfound) = Data_value(idx(wasfound));
vals.m_isTouchingGround = f_values;
% vals.Properties.VariableDescriptions has the header
%% Longitudial model 
% Stevens:  x=[alpha, q, v_t, phi] u = [delta_e, delta_t]
m_airspeed_MPS = 1;
m_inclination_DEG = 9; % NOTE: we don't have AoA directly...
m_pitchRate_DEGpSEC = 5;
% Gotta get quats
m_orientationQuaternion_W = 43;
m_orientationQuaternion_X = 40;
m_orientationQuaternion_Y = 41;
m_orientationQuaternion_Z = 42;

%% Lateral model
% Stevens: x= [beta, roll, roll_rate, yaw_rate] u = [delta_aileron, delta_rudder]
m_roll_DEG = 10;
m_rollRate_DEGpSEC = 6;
m_yawRate_DEGpSEC = 7;

%% Common
% Time
m_currentPhysicsTime_SEC = 38;
% Inputs
rc_channel_0 = 44;
rc_channel_1 = 45;
rc_channel_2 = 46;
rc_channel_3 = 47;
rc_channel_4 = 48;
rc_channel_5 = 49;
rc_channel_6 = 50;
rc_channel_7 = 51;
% Others


% Get variables
yawRate_full = table2array(vals(:,m_yawRate_DEGpSEC));
vt_full = table2array(vals(:,m_airspeed_MPS));
roll_full = table2array(vals(:,m_roll_DEG));
incl_full = table2array(vals(:, m_inclination_DEG));
q_full = table2array(vals(:,m_pitchRate_DEGpSEC));
quat_w = table2array(vals(:,m_orientationQuaternion_W));
quat_x = table2array(vals(:,m_orientationQuaternion_X));
quat_y = table2array(vals(:,m_orientationQuaternion_Y));
quat_z = table2array(vals(:,m_orientationQuaternion_Z));
t_full = table2array(vals(:,m_currentPhysicsTime_SEC));
rc_0 = table2array(vals(:,rc_channel_0)); % roll - aileron input
rc_1 = table2array(vals(:,rc_channel_1)); % pitch - elevator input
rc_2 = table2array(vals(:,rc_channel_2)); % throttle - throttle input
rc_3 = table2array(vals(:,rc_channel_3)); % yaw - rudder input
rc_4 = table2array(vals(:,rc_channel_4)); % Null
rc_5 = table2array(vals(:,rc_channel_5)); % Null
rc_6 = table2array(vals(:,rc_channel_6)); % gear
rc_7 = table2array(vals(:,rc_channel_7)); % Null

%% Plot Controls and States
subplot(2,4,1)
plot(t_full,rc_0)
title("Aileron Input")

subplot(2,4,2)
plot(t_full,rc_1)
title("Elevator Input")

subplot(2,4,3)
plot(t_full,rc_2)
title("Throttle Input")

subplot(2,4,4)
plot(t_full,rc_3)
title("Rudder Input")

subplot(2,4,5)
plot(t_full,roll_full)
title("Roll")

subplot(2,4,6)
plot(t_full,q_full)
title("Pitch Rate d/s")

subplot(2,4,7)
plot(t_full,vt_full)
title("Airspeed")

subplot(2,4,8)
plot(t_full,yawRate_full)
title("Yaw Rate")


%% Select Data the User would Like

% to do 
% Handle this so we are saving the right thing

answer = questdlg('Which plot would you like to select time stamps from?', ...
	'Plot Menu', ...
	'Airspeed','Elevator Input','Roll','Airspeed');
% Handle response
switch answer
    case 'Airspeed'
        disp([answer ' coming right up.'])
        dispPlot = 1;
    case 'Elevator Input'
        disp([answer ' coming right up.'])
        dispPlot = 2;
    case 'Roll'
         disp([answer ' coming right up.'])
        dispPlot = 0;
end




%% Handle data and user input

% Airspeed
if dispPlot == 1
    t = t_full;
    y = vt_full;
    names = {'Airspeed'};
    % Select the data
    figure; 
    plotData = plot(t_full, vt_full);
    brush on
    disp('Hit Enter in comand window when done brushing')
    pause;
    for k = 1:numel(plotData)
        % Check that the property is valid for that type of object
        % Also check if any points in that object are selected
        if isprop(plotData(k),'BrushData') && any(plotData(k).BrushData)
            % Output the selected data to the base workspace with assigned name
            ptsSelected = logical(plotData(k).BrushData.');
            data = [t(ptsSelected) y(ptsSelected,k)];
            assignin('base',names{k},data)
        end
    end
    init = Airspeed(1,1);
    info = size(Airspeed); 
    info = info(1); 
    exit = Airspeed(info,1);

    % Finding Time, set initial vals
    p = 1; 
    q = 1;
    w = 1;

    % Find the initial value to delete
    time=table2array(vals(:,m_currentPhysicsTime_SEC));
    for i=1:length(time(:,1))
        if (time(p,:)~= init)
            p=p+1;
        elseif(time(p,:)== init)
            break
        end
            i=i+1;
    end
    % Find the exit value to delete
    time=table2array(vals(:,m_currentPhysicsTime_SEC));
    for u=1:length(time(:,1))
        if (time(w,:)~= exit)
             w = w +1;
        elseif(time(w,:)== exit)
            break
        end
            u=u+1;
    end

elseif dispPlot == 2
    t = t_full;
    y = rc_1;
    names = {'Elevator'};
    % Select the data
    figure; 
    plotData = plot(t_full, rc_1);
    brush on
    disp('Hit Enter in comand window when done brushing')
    pause;
    for k = 1:numel(plotData)
        % Check that the property is valid for that type of object
        % Also check if any points in that object are selected
        if isprop(plotData(k),'BrushData') && any(plotData(k).BrushData)
            % Output the selected data to the base workspace with assigned name
            ptsSelected = logical(plotData(k).BrushData.');
            data = [t(ptsSelected) y(ptsSelected,k)];
            assignin('base',names{k},data)
        end
    end
    init = Elevator(1,1);
    info = size(Elevator); 
    info = info(1); 
    exit = Elevator(info,1);

    % Finding Time, set initial vals
    p = 1; 
    q = 1;
    w = 1;

    % Find the initial value to delete
    time=table2array(vals(:,m_currentPhysicsTime_SEC));
    for i=1:length(time(:,1))
        if (time(p,:)~= init)
            p=p+1;
        elseif(time(p,:)== init)
            break
        end
            i=i+1;
    end
    % Find the exit value to delete
    time=table2array(vals(:,m_currentPhysicsTime_SEC));
    for u=1:length(time(:,1))
        if (time(w,:)~= exit)
             w = w +1;
        elseif(time(w,:)== exit)
            break
        end
            u=u+1;
    end
elseif dispPlot == 0
    
    t = t_full;
    y = roll_full;
    names = {'Roll_Plot'};
    % Select the data
    figure; 
    plotData = plot(t_full, roll_full);
    brush on
    disp('Hit Enter in comand window when done brushing')
    pause;
    for k = 1:numel(plotData)
        % Check that the property is valid for that type of object
        % Also check if any points in that object are selected
        if isprop(plotData(k),'BrushData') && any(plotData(k).BrushData)
            % Output the selected data to the base workspace with assigned name
            ptsSelected = logical(plotData(k).BrushData.');
            data = [t(ptsSelected) y(ptsSelected,k)];
            assignin('base',names{k},data)
        end
    end
    init = Roll_Plot(1,1);
    info = size(Roll_Plot); 
    info = info(1); 
    exit = Roll_Plot(info,1);

    % Finding Time, set initial vals
    p = 1; 
    q = 1;
    w = 1;

    % Find the initial value to delete
    time=table2array(vals(:,m_currentPhysicsTime_SEC));
    for i=1:length(time(:,1))
        if (time(p,:)~= init)
            p=p+1;
        elseif(time(p,:)== init)
            break
        end
            i=i+1;
    end
    % Find the exit value to delete
    time=table2array(vals(:,m_currentPhysicsTime_SEC));
    for u=1:length(time(:,1))
        if (time(w,:)~= exit)
             w = w +1;
        elseif(time(w,:)== exit)
            break
        end
            u=u+1;
    end
end 

%% Removing non-selected time

takeOffTime = p;
totalTime   = u-p; 

% Read CSV Data (this can also read xlsx, txt, and dat  files)
maxSize = size(vals);

% Delete data for the first 10 data points
vals([1:takeOffTime],:) = [];
maxSize = size(vals);

% Set the size to the number of rows in the data
maxSize = maxSize(1);

% Trim the last 10 Values
deleteSize = totalTime;
vals([deleteSize+1:maxSize],:) = [];

% Clip variables
yawRate_full = table2array(vals(:,m_yawRate_DEGpSEC));
vt_full = table2array(vals(:,m_airspeed_MPS));
roll_full = table2array(vals(:,m_roll_DEG));
incl_full = table2array(vals(:, m_inclination_DEG));
q_full = table2array(vals(:,m_pitchRate_DEGpSEC));
quat_w = table2array(vals(:,m_orientationQuaternion_W));
quat_x = table2array(vals(:,m_orientationQuaternion_X));
quat_y = table2array(vals(:,m_orientationQuaternion_Y));
quat_z = table2array(vals(:,m_orientationQuaternion_Z));
t_full = table2array(vals(:,m_currentPhysicsTime_SEC));
rc_0 = table2array(vals(:,rc_channel_0)); % roll - aileron input
rc_1 = table2array(vals(:,rc_channel_1)); % pitch - elevator input
rc_2 = table2array(vals(:,rc_channel_2)); % throttle - throttle input
rc_3 = table2array(vals(:,rc_channel_3)); % yaw - rudder input
rc_4 = table2array(vals(:,rc_channel_4)); % Null
rc_5 = table2array(vals(:,rc_channel_5)); % Null
rc_6 = table2array(vals(:,rc_channel_6)); % gear
rc_7 = table2array(vals(:,rc_channel_7)); % Null

%move to an array
vals = table2array(vals);

prompt = {'What do you want to rename the data to?'};
dlgtitle = 'Rename Data File';
definput = {'clippedData'};
dims = [1 40];
dataName = inputdlg(prompt,dlgtitle,dims,definput);
save(dataName{1,1}, 'vals'); 

close all
% To do 
% Handle Ulog File 

% To do
% Clean up code


