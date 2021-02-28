% D1= CXq; D2= CYr; D3 = CYp; D4= CZq; D5= Clr; D6= Clp
% D7= Cmq; D8= Cnr; D9 = Cnp;

function CXX= CX(Alpha,el)
% Body-axis X Force
a=[-.099 -.081 -.081 -.063 -.025 .044 .097 .113 .145 .167 .174 .166
-.048 -.038 -.040 -.021 .016 .083 .127 .137 .162 .177 .179 .167
-.022 -.020 -.021 -.004 .032 .094 .128 .130 .154 .161 .155 .138
-.040 -.038 -.039 -.025 .006 .062 .087 .085 .100 .110 .104 .091
-.083 -.073 -.076 -.072 -.046 .012 .024 .025 .043 .053 .047 .040]';
% This has been checked for accuracy with the Stevens Table
s=.2*Alpha;
k=fix(s);
if(k<=-2),k=-1;end
if(k>=9),k=8;end
da=s-k;
l=k+fix(1.1*sign(da));
s=el/12;
m=fix(s);
if(m<=-2),m=-1;end
if(m>=2),m=1;end
de=s-m;
n=m+fix(1.1*sign(de));
k=k+3;
l=l+3;
m=m+3;
n=n+3;
t=a(k,m);
u=a(k,n);
v=t+abs(da)*(a(l,m)-t);
w=u+abs(da)*(a(l,n)-u);
CXX = v+(w-v)*abs(de); %.0574 

end