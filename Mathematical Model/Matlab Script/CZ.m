function czz = CZ(Alpha,Beta,EL)
% Body-axis Z Force
a= [.770 .241 -.100 -.416 -.731 -1.053 -1.366 -1.646 -1.917 -2.120 -2.248 -2.229]';
s =.2*Alpha;
% truncates s to a decimal nearest 0
k = fix(s);
if(k<=-2), k=-1;end
if(k>=9), k=8;end
DA = s-k;
l=k+fix(1.1*sign(DA));
l=l+3;
k=k+3;
s=a(k)+abs(DA)*(a(l)-a(k));
czz=s*(1-(Beta/57.3)^2)-.19*(EL/25);
% Checked for accuracy with the Stevens Model
% This has been double checked for accuracy
end