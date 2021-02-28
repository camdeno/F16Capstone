function [AMACH,QBAR] = ADC(VT,alt) % air data computer
R0   = 2.377e-3; % sea-level density
TFAC = 1.0 - (0.703e-5 * alt);
T    = 519.0 * TFAC; % temperature
if (alt >= 35000.0)
    T= 390.0;
end
RHO = R0 * (TFAC^4.14); % density
AMACH = VT/sqrt(1.4*1716.3*T); % Mach number
QBAR  = 0.5*RHO*VT*VT; % dynamic pressure
% PS = 1715.0 * RHO * T ! static pressure
return
% Checked for accuracy with the stevens model.
end