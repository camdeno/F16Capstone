function CY = CY(Beta,Ail,Rdr)
% Body-axis y Force
CY = -.02*Beta+.021*(Ail/20)+.086*(Rdr/30);
% Checked for accuracy with the stevens model.
end