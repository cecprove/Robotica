%% Prova con toad
t=(0:0.1:20);
u=[20 20 30 20 0 10 20 0 pi/6 0]; % posizione iniziale, posizione finale, tempo iniziale
% per il primo tratto, trempo finale per il primo tratto, tempo finale,
% orientamento iniziale , orientamento finale, variabile tempo
% per come è scritto il codice dovremmo invertire i pedici
c1=[25 34.1];
c2=[25 11];
r1=15;
r2=10;
for i=1:length(t)
    u(10)=t(i);
    [xd(i).traiettoria,xd(i).derivata,phi(i).orientamento,phi(i).derivata]=...
        planner_TOAD(u,c1,c2,r1,r2);
end

figure()
for i=1:length(t)
x(i)=xd(i).traiettoria(1);
y(i)=xd(i).traiettoria(2);
end
axis equal
plot(x',y')
axis equal
% per l'orientamento

figure()
for i=1:length(t)
a(i)=phi(i).orientamento;
%a(i)=phid(i).orientamento;
end
plot(t',a')

