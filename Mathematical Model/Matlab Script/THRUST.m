function THRUST = THRUST(POW,alt,RMACH) %Engine thrust model
% adding a check for neg velocity
    if RMACH < 0
        RMACH=0;
    end

a=[1060 670 880 1140 1500 1860
635 425 690 1010 1330 1700
60 25 345 755 1130 1525
-1020 -710 -300 350 910 1360
-2700 -1900 -1300 -247 600 1100
-3600 -1400 -595 -342 -200 700]';

b=[12680 9150 6200 3950 2450 1400
12680 9150 6313 4040 2470 1400
12610 9312 6610 4290 2600 1560
12640 9839 7090 4660 2840 1660
12390 10176 7750 5320 3250 1930
11680 9848 8050 6100 3800 2310]';

c=[20000 15000 10800 7000 4000 2500
21420 15700 11225 7323 4435 2600
22700 16860 12250 8154 5000 2835
24240 18910 13760 9285 5700 3215
26070 21075 15975 11115 6860 3950
28886 23319 18300 13484 8642 5057]';


if(alt<0),alt=0.01;end
h=.0001*alt;
i=fix(h);
if(i>=5),i=4;end
dh=h-i;
rm=5*RMACH;
m=fix(rm);
if(m>=5);m=4;end

dm=rm-m;
cdh=1-dh;

i=i+1;
m=m+1;

s=b(i,m)*cdh+b(i+1,m)*dh;
t=b(i,m+1)*cdh+b(i+1,m+1)*dh;
tmil=s+(t-s)*dm;
if(POW<50)
s=a(i,m)*cdh+a(i+1,m)*dh;
t=a(i,m+1)*cdh+a(i+1,m+1)*dh;
tidl=s+(t-s)*dm;
THRUST=tidl+(tmil-tidl)*POW*.02;
else
s=c(i,m)*cdh+c(i+1,m)*dh;
t=c(i,m+1)*cdh+c(i+1,m+1)*dh;
tmax=s+(t-s)*dm;
THRUST=tmil+(tmax-tmil)*(POW-50)*.02;
end
% Checked for accuracy with the stevens model.

 