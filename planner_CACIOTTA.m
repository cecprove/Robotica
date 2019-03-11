%This function calculates the actual desired robot position, using a third
%order polynomial function. 
%Input parameters are initial position (Pi), final position (Pf), initial 
%time (ti), final time (tf) and current time (t) and position of the center of
%the circels (c2).
%Output variables are robot desired position (XYd) and velocity (XYddot).

function [xd,xdd]=planner_CACIOTTA(u,c1,c2,r1,r2)
pf=u(1:2);
p_i=u(3:4);
ti=u(5);
tf=u(6);
tf2=u(7);
t=u(8);
alfa_2=deg2rad(270);
if t<=tf
A_2=[ti^3 ti^2 ti 1;
   tf^3 tf^2 tf 1;
   3*ti^2 2*ti 1 0;
   3*tf^2 2*tf 1 0];
B_2=[0;r2*alfa_2;0;0];
a_2=inv(A_2)*B_2;
s=a_2(1)*t^3+a_2(2)*t^2+a_2(3)*t+a_2(4);
sdot=3*a_2(1)*t^2+2*a_2(2)*t+a_2(3);
R=[cos(pi/4) sin(pi/4);
    -sin(pi/4) cos(pi/4)];
if t<ti
    XYd=p_i;
    XYddot=[0;0];
else
    if t<=tf
    XYd=(c2'+R*[r2*cos(s/r2) r2*sin(s/r2)]')';
    XYddot=(R*[-sin(s/r2) cos(s/r2)]')';
    else
    XYd=pf;
    XYddot=[0;0];
    end
end
else
A_1=[tf^3 tf^2 tf 1;
   tf2^3 tf2^2 tf2 1;
   3*tf^2 2*tf 1 0;
   3*tf2^2 2*tf2 1 0];
B_1=[0;r1*pi;0;0];
a_1=inv(A_1)*B_1;
s=a_1(1)*t^3+a_1(2)*t^2+a_1(3)*t+a_1(4);
sdot=3*a_1(1)*t^2+2*a_1(2)*t+a_1(3);
R1=[cos(pi) sin(pi);
    -sin(pi) cos(pi)];
    if t<=tf2
    XYd=(c1'+R1*[r1*cos(s/r1) r1*sin(s/r1)]')';
    XYddot=(R1*[-sin(s/r1) cos(s/r1)]')';
    else
    XYd=p_i;
    XYddot=[0;0];
    end
end    
  xd=XYd;
  xdd=XYddot;
 
end