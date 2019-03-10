function output=check_links_dimensions_4Dof(p,theta,a1,a2,a3,a4,lim)
%& da in uscita un valore booleano e fa il check il controllo di quanto
%dato in ingresso
q=[]; 
output=true;
if(isempty(p)==0 && isempty(theta)==0)%%verifico se i due vettori non sono vuoti
    %& metto i primi due valori del vettore che ho messo
    p_=p(1,:);
    theta_=theta(1);
    %& li scorro per vedere la lunghezza e poi vedo se questi valori sono
    %compatibili con quelli del nostro workspace
    if(length(p)>1 && length(theta)>1)%%passo all'elemento successivo
        p=p(2:end,:); %& è come se eliminassi il primo elemento?
        theta=theta(2:end);
    else 
        p=[];
        theta=[];
    end
    
    %& blocco di inversione analitica per passare dallo spazio operativo a
    %quello dei giunti. In Q ho la configurazione degli angoli di giunto
    %che ci aspettiamo
    
    %& mia aggiunta per far funzionare l'algoritmo
    if isempty(q)
        q=[deg2rad(46.8);deg2rad(-30.4);deg2rad(-72.3);deg2rad(20)]; 
    end % quindi al primo colpo usa qi poi userà le q già ottenute
    
    Q=analitycal_IK_4DoF(p_,theta_,a1,a2,a3,a4,q); % qi l'ho aggiunto io per l'algoritmo di inversione
    %& verifica se Q è un vettore vuoto e se appartine e a quell'intervallo
    %impostato inizialmente
    if (isempty(Q)==0 && Q(1)>lim(1,1) && Q(1)<lim(1,2) ...
                      && Q(2)>lim(2,1) && Q(2)<lim(2,2) ...
                      && Q(3)>lim(3,1) && Q(3)<lim(3,2) ...
                      && Q(4)>lim(4,1) && Q(4)<lim(4,2))
       % output=true;
        output=output && check_links_dimensions_4DoF(p,theta,a1,a2,a3,a4,lim);%& è una funzione di tipo ricorsivo 
        %& poichè lo devo fare per tutte le p e theta e vedo se è
        %accetabile attraverso la scrittura output && check.. Se la
        %funzione mi restituisce falso l'and non funziona più 
    else
        output=false;%se la condizione non è verificata,l'output della funzione sarà false
    end
end



end