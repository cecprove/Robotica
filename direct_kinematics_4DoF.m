% calcolo cinematica diretta per manipolatore planare a 4DoF
function X=direct_kinematics_4DoF(q1,q2,q3,q4,a1,a2,a3,a4)


X=[a1*cos(q1)+a2*cos(q1+q2)+a3*cos(q1+q2+q3)+a4*cos(q1+q2+q3+q4);
    a1*sin(q1)+a2*sin(q1+q2)+a3*sin(q1+q2+q3)+a4*sin(q1+q2+q3+q4);
    q1+q2+q3+q4];
end