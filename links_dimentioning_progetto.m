%DIMENSIONAMENTO LINK -
%
% Other m-files required: analitycal_IK_3DoF.m, check_links_dimensions.m,
% direct_kinematics_3DoF.m
% MAT-files required: none
%-Dati in cm-
% Author: Clemente Lauretti, Francesco Scotto di Luzio, Francesca Cordella
%
% email: c.lauretti@unicampus.it, f.scottodiluzio@unicampus.it,
% f.cordella@unicampus.it
% Website: http://www.unicampus.it
% March 2019; Last revision: 03-06-2019
%------------- BEGIN CODE --------------

clear all
close all
clc

%Limiti di giunto del manipolatore
joint_lim=[deg2rad(-90) deg2rad(90);
    deg2rad(-90) deg2rad(90);
    deg2rad(-90) deg2rad(90)
    deg2rad(-90) deg2rad(90)];
% &contiene un insieme di angoli e rappresentano gli estremi inferiore e
%superiore oltre il quale il giunto non può muoversi. Sono epsressi in
%radianti

%Traiettoria di esempio nello spazio operativo
p=[ 28 28;
    29 29;
    30 30;
    31 31;
    32 31;
    33 33];

theta=[0.1;
    0.2;
    0.3;
    0.4;
    0.5;
    0.6];
%& in questo caso e anche nel nostro siamo nel piano ma in questo caso
%abbiamo 3gdl qindi abbiamo un vettore che contiene le x e le y ( le due
%colonne) per ogni istante di tempo e poi per l'orientamento abbiamo theta

%Grandezza minima/massima dei link
link_lim=[1 20;
    1 20;
    1 20; 
    1 20];
%& Scelgo le lunghezze massime e minime dei miei link . Queste sono fissate
%( le lunghezze sono in cm) Dobbiamo lascirlse cosi come sono

%Range di variazione dei link nel metodo di ottimizzazione
resolution=0.5;
%& corrisponde al valore minimo che l'algorimo andrà a testare in base al
%workspace che ho. Se non va bene la combinazione va scartata. per
%scegliere quale tra le varie combinazione utilizzo una funzione di costo
%come ad esempio quella che mi fa risprmiare materiale di stampa quindi la
%somma dei tre lonk è minima ma non basta es uno da 1cm e altri molto
%grandi--> sconsigliata. Quindi media dei tre link e poi vedo quanto ci si
%discosta da questa media

%Vettori vuoti
links=[];
links_sum=[];
links_diff=[];
links_=[];

%3 cicli for innestati per scorrere i vettori
%& i tre indici del for sono la lunghezza dei tre link e li faccio variare
%tra il minimo e il massimo con un passo che è pari alla risoluzione
for a1=link_lim(1,1):resolution:link_lim(1,2)
    for a2=link_lim(2,1):resolution:link_lim(2,2)
        for a3=link_lim(3,1):resolution:link_lim(3,2)
            for a4=link_lim(4,1):resolution:link_lim(4,2)
                
                check=check_links_dimensions_4DoF(p,theta,a1,a2,a3,a4,joint_lim);
            

                %Ciclo if
                %& se check è vero inserisco in link a1,a2,a3 e se voglio
                %conservare anche quelli precedenti (dove c'è il check vero)
                %scrivo links=[links; a1...]
                if(check)
                    links=[links; a1, a2, a3, a4];
                    %& devo poi realizzare quella funzione che mi va a
                    % minimizzare quelle funzioni di costo che ho scelto prima

                    %& prima funzione di costo è la somma dei link quindi creo
                    %una variabile contenente le somme di tutti i link
                    links_sum=[links_sum;a1+a2+a3+a4];
                    %& mi permettono di trovare il massimo delle combinazioni
                    %dei tre link
                    max_1234=max(abs(a1-(a1+a2+a3+a4)/4),abs(a2-(a1+a2+a3+a4)/4),abs(a3-(a1+a2+a3+a4)/4),abs(a4-(a1+a2+a3+a4)/4));

                    links_diff=[links_diff;max_1234];
                end
                %& c'è _ .Qui metto tutte le combinazioni che ho trovato anche
                %quelle che non vanno bene
                links_=[links_; a1, a2, a3, a4];
            end
        end
    end
end

%Vengono applicate delle funzioni di costo
%& tipo deviazione stadard
links_sum=links_sum-mean(links_sum);
links_diff=links_diff-mean(links_diff);
%& normalizzo solitamnete divido tutti gli elementi del vettore per la
%differenza tra il massimo e il minimo e cosi poi tutti gli elementi del
%vettore varieranno tra 0 e 1.
links_sum_norm=links_sum/abs((max(links_sum)-min(links_sum)));
links_diff_norm=links_diff/abs((max(links_diff)-min(links_diff)));

cost_function= links_sum_norm + links_diff_norm;

figure(1)
plot(links_sum,'-r','LineWidth',3)
hold on
plot(links_diff,'-b','LineWidth',3)

figure(2)
plot(cost_function,'-g','LineWidth',3)
hold on
plot(links_sum_norm,'-r','LineWidth',3)
hold on
plot(links_diff_norm,'-b','LineWidth',3)

%& ho una funzione di costo la voglio minimizzare quindi prendo il minimo.
%La tilde è un tool di matlab mi dice che voglio solo l'indice del valore
%della funzione, stA PER non considerare tutto ciò che sta prima poichè min
%fornisce sia il valore minimo che l'indice e noi gli diciamo di prendere
%solo l'indice.
[~, correct_ind] = min(cost_function);
%& cosi salvo la combinazione corretta che meglio minimizza il funzionale
%di costo
link=links(correct_ind,:)

