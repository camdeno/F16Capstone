% Flight Log Analysis
% Cam Osborn
% Loads flight log, maps the signals, reads rc controller data, extracts
% data as a timetable 

% Input Flight Log
filename = 'log_25_2021-5-19-10-32-00.ulg';
ulog = ulogreader(filename);

% Use Custom Signal Mapping
FlightObj = flightLogSignalMapping('ulog');

% Check Mapped Signals
info(FlightObj,"Signal")

% Read Messages
msg = readTopicMsgs(ulog);

% Set Time interval
d1 = ulog.StartTime;
d2 = d1 + duration([0 0 55],'Format','hh:mm:ss.SSSSSS');

% read RC Controller Data
rcController = readTopicMsgs(ulog,'TopicNames',{'manual_control_setpoint'}, ... 
'InstanceID',{0},'Time',[d1 d2]); 

% Extract Manual Control Inputs from the joystick / radio
manualInputs = rcController.TopicMessages{1,1};

pitchControllerData = manualInputs(:,1);
rollControllerDataData = manualInputs(:,2);
yawControllerData = manualInputs(:,3);

 