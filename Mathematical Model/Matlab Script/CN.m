function cnn=CN(Alpha,Beta)
% Yawing moment
a=[0 0 0 0 0 0 0 0 0 0 0 0
.018 .019 .018 .019 .019 .018 .013 .007 .004 -.014 -.017 -.033
.038 .042 .042 .042 .043 .039 .030 .017 .004 -.035 -.047 -.057
.056 .057 .059 .058 .058 .053 .032 .012 .002 -.046 -.071 -.073
.064 .077 .076 .074 .073 .057 .029 .007 .012 -.034 -.065 -.041
.074 .086 .093 .089 .080 .062 .049 .022 .028 -.012 -.002 -.013
.079 .090 .106 .106 .096 .080 .068 .030 .064 .015 .011 -.001]';
% Checked for accuracy with the stevens model.
s=.2*Alpha;
k=fix(s);
if(k<=-2),k=-1;end
if(k>=9),k=8;end
DA =s-k;
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
cnn=dum*sign(Beta);

% Checked for accuracy with the stevens model.
end