function RTAU = RTAU(DP) %used by function PDOT
if (DP <= 25.0) 
    RTAU=1.0;            %reciprocal time constant
elseif(DP>=50.0)
    RTAU=0.1;
else
    RTAU=1.9-.036*DP;
% Checked for accuracy with the stevens model.    
end
