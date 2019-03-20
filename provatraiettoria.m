%% Prova con toad
t=(0:0.1:20);
u=[20 20 30 20 0 10 20 0 pi/6 t]; 
c1=[25 49.5];
c2=[25 11];
r1=30;
r2=10;
for i=1:length(t)
    u(10)=t(i);
    [xd(i).traiettoria,xd(i).derivata,phi(i).orientamento,phi(i).derivata]=...
        planner_TOAD(u,c1,c2,r1,r2);
end

for i=1:length(t)
x(i)=xd(i).traiettoria(1);
y(i)=xd(i).traiettoria(2);
end
plot(x',y')
% per l'orientamento

for i=1:length(t)
a(i)=phi(i).orientamento;
end
plot(t',a')

