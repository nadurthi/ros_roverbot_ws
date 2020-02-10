umin=2000;
umax=4965;

t=0:0.05:2*60;
u1=sin(t)+cos(2*t)+2*sin(t/2)+cos(t/4);
u2=sin(t)+0.5*cos(2*t-0.5)+2*sin(t/2+1)+3*cos(t/4-1.5)-2*sin(3*t);
n=length(t);
u=[u1(1:floor(n/2)),u2(1+floor(n/2):end)];

plot(t,u)


U=u-min(u);
U=U/max(U);
U=umin+(umax-umin)*U;
plot(t,U)
U=floor(U');
U=floor(U');
fileID=fopen('InputCtrl.txt','w');
fprintf(fileID,'%d\n',U);
fclose(fileID);




