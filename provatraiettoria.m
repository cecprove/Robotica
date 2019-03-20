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
plot(x',y')
% per l'orientamento

for i=1:length(t)
a(i)=phi(i).orientamento;
end
plot(t',a')


%% Prova con toad
t=(0:0.1:20);
u=[20 20 30 20 0 10 20 t 0 pi/6]; 
% per come è scritto il codice dovremmo invertire i pedici
c1=[25 49.5];
c2=[25 11];
r1=30;
r2=10;
for i=1:length(t)
    u(10)=t(i);
    [xd(i).traiettoria,xd(i).derivata,phi(i).orientamento,phi(i).derivata]=...
        planner_TOAD(u,c1,c2,r1,r2);
end