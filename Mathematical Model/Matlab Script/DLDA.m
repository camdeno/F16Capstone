function DLDA = DLDA(Alpha,Beta)
%Rolling Moment Due to Ailerons
a=[-.041 -.052 -.053 -.056 -.050 -.056 -.082 -.059 -.042 -.038 -.027 -.017
-.041 -.053 -.053 -.053 -.050 -.051 -.066 -.043 -.038 -.027 -.023 -.016
-.042 -.053 -.052 -.051 -.049 -.049 -.043 -.035 -.026 -.016 -.018 -.014
-.040 -.052 -.051 -.052 -.048 -.048 -.042 -.037 -.031 -.026 -.017 -.012
-.043 -.049 -.048 -.049 -.043 -.042 -.042 -.036 -.025 -.021 -.016 -.011
-.044 -.048 -.048 -.047 -.042 -.041 -.020 -.028 -.013 -.014 -.011 -.010
-.043 -.049 -.047 -.045 -.042 -.037 -.003 -.013 -.010 -.003 -.007 -.008]';
% Checked for accuracy with the stevens model.
s=.2*Alpha;
k=fix(s);
if(k<=-2),k=-1;end
if(k>=9),k=8;end
DA=s-k;
l=k+fix(1.1*sign(DA));
s=.1*Beta;
m=fix(s);
if(m ==-3),m=-2;end
if(m >= 3),m=2;end
db=s-m;
n=m+fix(1.1*sign(db));
l=l+3;
k=k+3;
 m=m+4;
 n=n+4;
if (m >= 7), m=7; end 
if (m < 1 ), m=1; end
if (n < 1 ), n=1; end
if (k >= 12), k=12; end
t=a(k,m);
u=a(k,n);
v=t+abs(DA)*(a(l,m)-t);
w=u+abs(DA)*(a(l,n)-u);
% Checked for accuracy with the stevens model.
DLDA = v+(w-v)*abs(db);
% Double checked for accuracy with the stevens model. 