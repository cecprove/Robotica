function Q=analitycal_IK_4DoF(p,theta,a1,a2,a3,a4)
%& per passara dallo spazio operativo a quello dei giunti

%Applicazione del problema cinematico inverso (Noti p e theta->si vuole ricavare Q)
%& guarda esempio del manipolatore a 3gdl fatto in classe
q4= deg2rad(7);
pw_x=p(1)-a3*cos(theta-q4)-a4*cos(theta);
pw_y=p(2)-a3*sin(theta-q4)-a4*sin(theta);

c2=(pw_x^2+pw_y^2-a1^2-a2^2)/(2*a1*a2);
s2=+sqrt(1-c2^2);

s1=((a1+a2*c2)*pw_y-a2*s2*pw_x)/(pw_x^2+pw_y^2);
c1=((a1+a2*c2)*pw_x+a2*s2*pw_y)/(pw_x^2+pw_y^2);

Q=[];
%& se non sono reali quella soluzione è da scartare
if (isreal(s1) && isreal(s2) && isreal(c1) && isreal(c2))
    q1=atan2(s1,c1);
    q2=atan2(s2,c2);
    q3=theta-q1-q2-q4;
    %& ho una configurazione di angoli noti fi,theta e voglio
    %ricavare theta 1,2,3 ossia gli angoli di giunto. Se conosco tutto devo
    %riapplicare la cinematica diretta per vedere se la posizione che ho
    %trovato siano quelle desiderati poichè questi algoritmi possono creare
    %degli errori e quindi impongo che la differenza sia inferiore a e-3
    X = direct_kinematics_4DoF(q1,q2,q3,q4,a1,a2,a3,a4);%viene applicata la cinematica diretta
    
    if(abs(X-[p(1);p(2);theta])<1e-3)
        Q=[q1 q2 q3 q4];  
    end
    
end
end
