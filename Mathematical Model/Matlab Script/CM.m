function cmm = CM(Alpha,EL)
% Pitching moment
a=[.205 .168 .186 .196 .213 .251 .245 .238 .252 .231 .198 .192
.081 .077 .107 .110 .110 .141 .127 .119 .133 .108 .081 .093
-.046 -.020 -.009 -.005 -.006 .010 .006 -.001 .014 .000 -.013 .032
-.174 -.145 -.121 -.127 -.129 -.102 -.097 -.113 -.087 -.084 -.069 -.006
-.259 -.202 -.184 -.193 -.199 -.150 -.160 -.167 -.104 -.076 -.041 -.005]';
% Checked for accuracy with the stevens model.

s=.2*Alpha;
k=fix(s);
if(k<=-2),k=-1;end
if(k>=9),k=8;end
DA=s-k;
l=k+fix(1.1*sign(DA));
s=EL/12;
m=fix(s);
if(m<=-2),m=-1;end
if(m>=2),m=1;end
de=s-m;
n=m+fix(1.1*sign(de));
k=k+3;
l=l+3;
m = m+3;
n = n+3;
t = a(k,m);
u = a(k,n);
v= t+abs(DA)*(a(l,m)-t);
w=u+abs(DA)*(a(l,n)-u);
cmm=v+(w-v)*abs(de);
% Checked for accuracy with the stevens model
end