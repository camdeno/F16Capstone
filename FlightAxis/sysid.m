clear all; clc;
%vals = readtable('./13_05_2021__16_16_38.csv');
%save('./13_05_2021__16_16_38.mat','vals')
load('./13_05_2021__16_16_38.mat')

%% 
header = ["m_airspeed_MPS","m_altitudeASL_MTR","m_altitudeAGL_MTR","m_groundspeed_MPS","m_pitchRate_DEGpSEC","m_rollRate_DEGpSEC","m_yawRate_DEGpSEC","m_azimuth_DEG","m_inclination_DEG","m_roll_DEG","m_aircraftPositionX_MTR","m_aircraftPositionY_MTR","m_velocityWorldU_MPS","m_velocityWorldV_MPS","m_velocityWorldW_MPS","m_velocityBodyU_MPS","m_velocityBodyV_MPS","m_velocityBodyW_MPS","m_accelerationWorldAX_MPS2","m_accelerationWorldAY_MPS2","m_accelerationWorldAZ_MPS2","m_accelerationBodyAX_MPS2","m_accelerationBodyAY_MPS2","m_accelerationBodyAZ_MPS2","m_windX_MPS","m_windY_MPS","m_windZ_MPS","m_propRPM","m_heliMainRotorRPM","m_batteryVoltage_VOLTS","m_batteryCurrentDraw_AMPS","m_batteryRemainingCapacity_MAH","m_fuelRemaining_OZ","m_isLocked","m_hasLostComponents","m_anEngineIsRunning","m_isTouchingGround","m_currentPhysicsTime_SEC","m_currentPhysicsSpeedMultiplier","m_orientationQuaternion_X","m_orientationQuaternion_Y","m_orientationQuaternion_Z","m_orientationQuaternion_W","rc_channel_0","rc_channel_1","rc_channel_2","rc_channel_3","rc_channel_4","rc_channel_5","rc_channel_6","rc_channel_7","rc_channel_8","rc_channel_9","rc_channel_10","rc_channel_11"];

for idx = 1:size(header,2)
    expr = sprintf("%s = %d",header(idx),idx);
    eval(expr)
end

t = vals{:,m_currentPhysicsTime_SEC};
vt = vals{:,m_airspeed_MPS};
incl_deg = vals{:,m_inclination_DEG};
%% Plot
plot(t,incl_deg)
