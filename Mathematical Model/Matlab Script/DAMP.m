function D = DAMP(Alpha)

a=[-.267 -.110 .308 1.34 2.08 2.91 2.76 2.05 1.50 1.49 1.83 1.21
.882 .852 .876 .958 .962 .974 .819 .483 .590 1.21 -.493 -1.04
-.108 -.108 -.188 .110 .258 .226 .344 .362 .611 .529 .298 -2.27
-8.80 -25.8 -28.9 -31.4 -31.2 -30.7 -27.7 -28.2 -29.0 -29.8 -38.3 -35.3
-.126 -.026 .063 .113 .208 .230 .319 .437 .680 .100 .447 -.330
-.360 -.359 -.443 -.420 -.383 -.375 -.329 -.294 -.230 -.210 -.120 -.100
-7.21 -.540 -5.23 -5.26 -6.11 -6.64 -5.69 -6.00 -6.20 -6.40 -6.60 -6.00
-.380 -.363 -.378 -.386 -.370 -.453 -.550 -.582 -.595 -.637 -1.02 -.840
.061 .052 .052 -.012 -.013 -.024 .050 .150 .130 .158 .240 .150]';
% Checked for accuracy with the stevens model.
s=.2*Alpha;
k=fix(s);
if(k<=-2),k=-1;end
if(k>=9),k=8;end
da=s-k;
l=k+fix(1.1*sign(da));
k=k+3;
l=l+3;

for i=1:9
D(i)=a(k,i)+abs(da)*(a(l,i)-a(k,i));
end
% Checked for accuracy with the stevens model.
% Double Checked for accuracy 
end