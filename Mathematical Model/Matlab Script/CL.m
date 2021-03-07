  
function cll = CL(Alpha,Beta)
% Rolling moment
a=[0 0 0 0 0 0 0 0 0 0 0 0
-.001 -.004 -.008 -.012 -.016 -.019 -.020 -.020 -.015 -.008 -.013 -.015
-.003 -.009 -.017 -.024 -.030 -.034 -.040 -.037 -.016 -.002 -.010 -.019
-.001 -.010 -.020 -.030 -.039 -.044 -.050 -.049 -.023 -.006 -.014 -.027
.000 -.010 -.022 -.034 -.047 -.046 -.059 -.061 -.033 -.036 -.035 -.035
.007 -.010 -.023 -.034 -.049 -.046 -.068 -.071 -.060 -.058 -.062 -.059
.009 -.011 -.023 -.037 -.050 -.047 -.074 -.079 -.091 -.076 -.077 -.076]';
% Checked for accuracy with the stevens model.
s=.2*Alpha;
k=fix(s);
if(k<=-2),k=-1;end
if(k>=9),k=8;end
DA=s-k;
l=k+fix(1.1*sign(DA));
s=.2*abs(Beta);
m=fix(s);
if(m==0),m=1;end
if(m>=6),m=5;end
db=s-m;
n=m+fix(1.1*sign(db));
l=l+3;
k=k+3;
m=m+1;
n=n+1;
t=a(k,m);
u=a(k,n);
v=t+abs(DA)*(a(l,m)-t);
w=u+abs(DA)*(a(l,n)-u);
dum=v+(w-v)*abs(db);
cll=dum*sign(Beta);
% Checked for accuracy with the stevens model.
end