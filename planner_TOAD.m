%This function calculates the actual desired robot position, using a third
%order polynomial function. 
%Input parameters are initial position (Pi), final position (Pf), initial 
%time (ti), final time (tf) and current time (t) and position of the center of
%the circels (c2).
%Output variables are robot desired position (XYd) and velocity (XYddot).

function [xd,xdd,phi,phit]=planner_TOAD(u,c1,c2,r1,r2)
%mi definisco i parametri da utilizzare per la pianificazione della
%traiettoria e dell'orientamento:
pf=u(1:2); %punto finale per il primo tratto, coincide con il punto A
p_i=u(3:4); %punto iniziale per il primo tratto, coincide con il punto B
ti=u(5); %tempo iniziale
tf=u(6); %tempo finale per il primo tratto
tf2=u(7); %tempo finale per il secondo tratto
phi_i=u(8); %phi iniziale per il primo tratto, ovvero in B
phi_f=u(9); %phi finale per il primo tratto, ovvero in A
t=u(10); %variabile temporale

alfa_2=deg2rad(180); %angolo di rotazione per il primo tratto
 beta=deg2rad(268.84);

if t<=tf %se ci troviamo ad un tempo inferiore di tf, dobbiamo pianificare
    %la traiettoria circolare del primo tratto, con raggio 'r1' e centro
    %'c1'
    A_2=[ti^3 ti^2 ti 1;
       tf^3 tf^2 tf 1;
       3*ti^2 2*ti 1 0;
       3*tf^2 2*tf 1 0]; %definisco la matrice dei termini noti temporali, derivante
    %dalle condizioni di definizione della legge oraria 's(t)', essa vale sia
    %per l'orientamento che per la traiettoria
    B_2=[0;r1*alfa_2;0;0]; %vettore dei termini noti per la traiettoria
    a_2=inv(A_2)*B_2; %coefficienti della legge oraria per la traiettoria
    
    s=a_2(1)*t^3+a_2(2)*t^2+a_2(3)*t+a_2(4); %legge oraria della traiettoria
    sdot=3*a_2(1)*t^2+2*a_2(2)*t+a_2(3); %derivata della legge oraria
    
    th=deg2rad(0);
     
    R=[cos(th) -sin(th);
        sin(th) cos(th)];
    %rotazione per la pianificazione di una traiettoria
    %circolare, sappiamo che l'equazione si riferisce ad una terna concentrica
    %a 'c1' e con angolo nullo quando ci troviamo nella situazione s(0)

    B_2phi=[phi_i;phi_f;0;0]; %vettore dei termini noti per l'orientamento
    a_2phi=inv(A_2)*B_2phi; %coefficienti della legge oraria per l'orientamento
    s_phi=a_2phi(1)*t^3+a_2phi(2)*t^2+a_2phi(3)*t+a_2phi(4); %legge oraria orientamento
    sdot_phi=3*a_2phi(1)*t^2+2*a_2phi(2)*t+a_2phi(3); %derivata legge oraria orientamento

    if t<=ti %se ci troviamo ad un tempo inferiore a quello iniziale allora ci troviamo nella
        %configurazione iniziale sia per la posizione che per l'orientamento
        XYd=p_i;
        XYddot=[0;0];
        phi_e=phi_i;
        phi_et=0;
    else
        if t<=tf %se ci troviamo tra ti e tf allora definisco la traiettoria
            %circolare e l'orientamento secondo una legge lineare con s_phi:
        XYd=(c1'+R*[r1*cos(s/r1); r1*sin(s/r1)])'; %è la x desiderata
        XYddot=(R*[-sin(s/r1); cos(s/r1)])';

        phi_e=phi_i+(s_phi/norm(phi_f-phi_i))*(phi_f-phi_i); %è la phi desiderata
        phi_et=(sdot_phi/norm(phi_f-phi_i))*(phi_f-phi_i);
    %     else %altrimenti rimango alla situazione finale, in realtà si può togliere questa parte
    %     XYd=pf;
    %     XYddot=[0;0];
    %     phi_e=phi_f;
    %     phi_et=0;
        end
    end

else %se ci troviamo ad un tempo maggiore di tf siamo passati a descrivere
    %la seconda traiettoria:
    A_1=[tf^3 tf^2 tf 1;
       tf2^3 tf2^2 tf2 1;
       3*tf^2 2*tf 1 0;
       3*tf2^2 2*tf2 1 0]; %definisco come prima la matrice A
    B_1=[0;r2*beta;0;0]; %vettore termini noti per la traiettoria 
    a_1=inv(A_1)*B_1; %coefficienti della legge oraria della traiettoria
    
    s=a_1(1)*t^3+a_1(2)*t^2+a_1(3)*t+a_1(4); %legge oraria traiettoria
    sdot=3*a_1(1)*t^2+2*a_1(2)*t+a_1(3); %derivata legge oraria
    
     gamma=deg2rad(135.58);
    
    R1=[cos(gamma) -sin(gamma);
        sin(gamma) cos(gamma)]; %& pi %matrice di rotazione per rimetterci nelle condizioni
    %per la pianificazione della traiettoria con centro 'c2' e raggio 'r2'

    B_1phi=[phi_i;phi_f;0;0]; %vettore termini noti per l'orientamento
    a_1phi=inv(A_1)*B_1phi; %coefficienti legge oraria orientamento
    s_phi=a_1phi(1)*t^3+a_1phi(2)*t^2+a_1phi(3)*t+a_1phi(4); %legge oraria orientamento
    sdot_phi=3*a_1phi(1)*t^2+2*a_1phi(2)*t+a_1phi(3); %derivata legge oraria orientamento

        if t<=tf2 %se ci troviamo ad un tempo inferiore di tf2 andiamo a pianificare la
            %traiettoria ed orientamento:
        XYd=(c2'+R1*[r2*cos(s/r2); r2*sin(s/r2)])'; %x desiderata
        XYddot=(R1*[-sin(s/r2); cos(s/r2)])';

        phi_e=phi_f+(s_phi/norm(phi_i-phi_f))*(phi_i-phi_f); %phi desiderata
        phi_et=(sdot_phi/norm(phi_i-phi_f))*(phi_i-phi_f);
        
        else %altrimenti rimaniamo alla configurazione da cui siamo partiti ovvero nel punto B
        XYd=p_i;
        XYddot=[0;0];
        phi_e=phi_i;
        phi_et=0;
        end
    end 

 %salvo i dati che voglio in uscita
  xd=XYd;
  xdd=XYddot;
  phi=phi_e;
  phit=phi_et;
 
end
