
function XY=direct_kinematics_4DoF(q1,q2,q3,q4,a1,a2,a3,a4)


XY=[a1*cos(q1)+a2*cos(q1+q2)+a3*cos(q1+q2+q3)+a4*cos(q1+q2+q3+q4);
    a1*sin(q1)+a2*sin(q1+q2)+a3*sin(q1+q2+q3)+a4*sin(q1+q2+q3+q4);
    q1+q2+q3+q4];
end