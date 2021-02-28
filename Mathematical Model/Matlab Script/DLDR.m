function DLDR=DLDR(Alpha,Beta)
%Yawing Moment due to rudder
a=[.005 .017 .014 .010 -.005 .009 .019 .005 -.000 -.005 -.011 .008
   .007 .016 .014 .014 .013 .009 .012 .005 .000 .004 .009 .007
   .013 .013 .011 .012 .011 .009 .008 .005 -.002 .005 .003 .005
   .018 .015 .015 .014 .014 .014 .014 .015 .013 .011 .006 .001
   .015 .014 .013 .013 .012 .011 .011 .010 .008 .008 .007 .003
   .021 .011 .010 .011 .010 .009 .008 .010 .006 .005 .000 .001
   .023 .010 .011 .011 .011 .010 .008 .010 .006 .014 .020 .000]';
% Checked for accuracy with the stevens model.
s=.2*Alpha;
k=fix(s);
if(k<=-2),k=-1;end
if(k>=9),k=8;end
DA=s-k;
l=k+fix(1.1*sign(DA));
s=.1*Beta;
m=fix(s);
if(m<=-3),m=-2;end
if(m>=3),m=2;end
db=s-m;
n=m+fix(1.1*sign(db));
l=l+3;
k=k+3;
m=m+4;
n=n+4;
t=a(k,m);
u=a(k,n);
v=t+abs(DA)*(a(l,m)-t);
w=u+abs(DA)*(a(l,n)-u);
DLDR=v+(w-v)*abs(db);
end