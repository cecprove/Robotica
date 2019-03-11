u(1:2)=[5 5];
u(3:4)=[10 5];
u(5)=0;
u(6)=10;
u(7)=20;
t=(0:0.1:20);
c1=[7.5 5];
c2=[7.5 7.5];
r1=2.5;
r2=sqrt(2*(2.5)^2);
for i=1:length(t)
    u(8)=t(i);
    [xd(i).traiettoria,xd(i).derivata]=planner_CACIOTTA(u,c1,c2,r1,r2);
end
for i=1:length(t)
x(i)=xd(i).traiettoria(1);
y(i)=xd(i).traiettoria(2);
end