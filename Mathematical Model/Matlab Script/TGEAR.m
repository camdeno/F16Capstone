function TGEAR = TGEAR(THTL) % Power command v. thtl. relationship
if(THTL<=0.77)
TGEAR = 64.94*THTL;
else
TGEAR = 217.38*THTL-117.38;
end
% Checked for accuracy with the stevens model.

 