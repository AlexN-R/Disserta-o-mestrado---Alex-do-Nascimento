clear all
close all

% Grandezas invariáveis
   
    %elétricas
   
    N=1;        %voltas das bobinas
   
    Ra=N*0.0062;
    Rb=N*0.0062;
    Rc=N*0.0062;
   
    R=Ra+0.008;
       
    La=N*3.5*(10^-6);
    Lb=N*3.5*(10^-6);
    Lc=N*3.5*(10^-6);
   
    L=La;
   
   
    Vdd=6;
   
    B=0.75;          %Tesla
    l=0.05;          % comprimento do fio
           
    %Mecânicas
   
    voltas=0;       % voltas mecânicas do rotor
   
    Cd=0.0001;        % coef. de atrito rotacional
   
    raio_mag=0.099;  % raio dos centro até os imãs
   
    raio_mot=0.115;  % raio do rotor
   
    raio_int=0.0895;  % raio interno do rotor
   
    raio_ext=0.20;  % raio interno do rotor
       
    I=1;           % Momento de inércia
   
    delta_theta=0.01; %graus

    polos=20;

    angulo_sloth=6;        %graus

    angulo_imas=360/polos; % graus

    abertura_ima= 14; %graus
   
    entrepolo=(angulo_imas-abertura_ima)/2;
   
    beta=0;            % [graus] defasagem de comutação
   
    posicao_HA=((0*angulo_sloth+beta)/delta_theta)+1;  % POSIÇÃO NUMÉRICA DO SENSORES HALL DA FASE A
   
    posicao_HB=(4*angulo_sloth+beta)/delta_theta;
   
    posicao_HC=(2*angulo_sloth+beta)/delta_theta;
   
%tempo

teste_loop=0;           %teste

    dt=0.0001;          %0.001;

    t=0:dt:30;

    nt=length(t);           %Nº de instantes

    theta_r=0:delta_theta:(360/(polos/2));          % Ângulo do rotor com relação ao estator
   
    n_tr=length(theta_r);   %Nº de passos em uma janela de 36º (em rad)
   
    counter=0;
   
   
% Grandezas no tempo
   
    % Elétricas
   
   
   
    Va=zeros(1,nt);
    Vb=zeros(1,nt);
    Vc=zeros(1,nt);
   
    Va(1)=0;
    Vb(1)=-Vdd;
    Vc(1)=Vdd;
   
    Vab=zeros(1,nt);
    Vcb=zeros(1,nt);

   
    Ea=zeros(1,nt);
    Eb=zeros(1,nt);
    Ec=zeros(1,nt);

    Eab=zeros(1,nt);
    Ecb=zeros(1,nt);
   
    dIa=zeros(1,nt);             % derivadas das corrente das fases
    dIb=zeros(1,nt);
    dIc=zeros(1,nt);
   
    Ia=zeros(1,nt);             % corrente das fases
    Ib=zeros(1,nt);
    Ic=zeros(1,nt);
   
       
   
    densi_fluxo_nom=zeros(1,n_tr);    %!!! DENSIDADE DE FLUXO
   
    Fluxo_a=zeros(1,nt);        % Fluxos por fase
    Fluxo_b=zeros(1,nt);
    Fluxo_c=zeros(1,nt);
   
       
    % Mecânicas
   
    theta=zeros(1,nt);          % Ângulo do rotor com relação ao estator
   
    theta_deg=zeros(1,nt);      % Theta em graus
   
    alpha=zeros(1,nt);          % aceleração angular
    w=zeros(1,nt);              % velocidade angular
   
    w(1)=0.0;              % velocidade angular
       
    Te=zeros(1,nt);             % Torque elétrico
    Tr=zeros(1,nt);             % Torque resultante
    Ta=zeros(1,nt);             % Torque atrito
    Tl=zeros(1,nt);             % Torque carga

% Calculo da densidade fluxo nominal no primeiro instante(FLUXO OU DENSIDADE DE FLUXO)
    for i=1:1:n_tr;

        if sin( (2*pi/360)*(polos/2)*theta_r(i) )> sin( (polos/2)*(2*pi/360)*(entrepolo) )

            densi_fluxo_nom(i)=1;

        elseif sin( (2*pi/360)*(polos/2)*theta_r(i) ) < -sin( (polos/2)*(2*pi/360)*(entrepolo) )

            densi_fluxo_nom(i)=-1;

        else

            densi_fluxo_nom(i)=0;

        end

    end

% Fluxo do instante zero  
 
  for j=1:1:3*((angulo_sloth)/ delta_theta);
     
    Fluxo_a(1)=Fluxo_a(1) + (pi/360)*delta_theta*B*( (raio_mot^2) - (raio_int^2) )*densi_fluxo_nom(j) ;        % Fluxos por fase
    Fluxo_b(1)=Fluxo_b(1) + (pi/360)*delta_theta*B*( (raio_mot^2) - (raio_int^2) )*densi_fluxo_nom(j +   ((angulo_sloth)/ delta_theta) );
    Fluxo_c(1)=Fluxo_c(1) + (pi/360)*delta_theta*B*( (raio_mot^2) - (raio_int^2) )*densi_fluxo_nom(j + 2*((angulo_sloth)/ delta_theta) );
   
  end
   

cont_comu=zeros(1,nt);                  %marcador de comutações
cont_comu(1)=0;


% Loop de tempo
for i=2:1:nt;
   
    %Input elétrico
               
               
                Va(i)=Va(i-1);% a princípio as tensões de entrada são as mesmas do loop anterior
                Vb(i)=Vb(i-1);
                Vc(i)=Vc(i-1);
               
            if (densi_fluxo_nom(posicao_HA)==1)&(densi_fluxo_nom(posicao_HB)==-1)&(densi_fluxo_nom(posicao_HC)==1);
               
                Va(i)=Vdd;
                Vb(i)=-Vdd;
                Vc(i)=0;
                cont_comu(i)=1;
            end

            if (densi_fluxo_nom(posicao_HA)==1)&(densi_fluxo_nom(posicao_HB)==-1)&(densi_fluxo_nom(posicao_HC)==-1);
               
                Va(i)=Vdd;
                Vb(i)=0;
                Vc(i)=-Vdd;
                cont_comu(i)=2;
            end
           
            if (densi_fluxo_nom(posicao_HA)==1)&(densi_fluxo_nom(posicao_HB)==1)&(densi_fluxo_nom(posicao_HC)==-1);
               
                Va(i)=0;
                Vb(i)=Vdd;
                Vc(i)=-Vdd;
                cont_comu(i)=3;
            end
           
            if (densi_fluxo_nom(posicao_HA)==-1)&(densi_fluxo_nom(posicao_HB)==1)&(densi_fluxo_nom(posicao_HC)==-1);
               
                Va(i)=-Vdd;
                Vb(i)=Vdd;
                Vc(i)=0;
                cont_comu(i)=4;
            end

            if (densi_fluxo_nom(posicao_HA)==-1)&(densi_fluxo_nom(posicao_HB)==1)&(densi_fluxo_nom(posicao_HC)==1);
               
                Va(i)=-Vdd;
                Vb(i)=0;
                Vc(i)=Vdd;
                cont_comu(i)=5;
            end
           
            if (densi_fluxo_nom(posicao_HA)==-1)&(densi_fluxo_nom(posicao_HB)==-1)&(densi_fluxo_nom(posicao_HC)==1);
               
                Va(i)=0;
                Vb(i)=-Vdd;
                Vc(i)=Vdd;
                cont_comu(i)=6;
            end
           
       
        Vcb(i)=Vc(i)-Vb(i);
        Vab(i)=Va(i)-Vb(i);
       
        %Calcula as derivadas das correntes

        dIa(i)=(-R/L)*Ia(i-1)+(2/(3*L))*(Vab(i)-Eab(i-1))-(1/(3*L))*(Vcb(i)-Ecb(i-1));

        dIc(i)=(-R/L)*Ic(i-1)+(2/(3*L))*(Vcb(i)-Ecb(i-1))-(1/(3*L))*(Vab(i)-Eab(i-1));

        dIb(i)=+(dIa(i)+dIc(i));
       
        %Calcula as correntes
       
        Ia(i)=Ia(i-1)+dIa(i)*dt;

        Ib(i)=Ib(i-1)+dIb(i)*dt;

        Ic(i)=Ic(i-1)+dIc(i)*dt;


       
    %Cálculo do Torque
       
        Te(i)=(N/2)*polos*( (raio_mot^2) - (raio_int^2) )*B*( densi_fluxo_nom(1)*Ia(i) + densi_fluxo_nom(angulo_sloth/delta_theta)*Ib(i) + densi_fluxo_nom(2*angulo_sloth/delta_theta)*Ic(i)  );
       
        Ta(i)=Cd*(w(i));
       
        Tr(i)= Te(i) - ( Ta(i) + Tl(i) );  %Cálculo do torque resultante
               
    % Cálculo da rotação
   
        alpha(i)= Tr(i)/I ;
       
        w(i)=w(i-1) + alpha(i)*dt;
       
        theta(i)=theta(i-1) + w(i)*dt;
       
        theta_deg(i) = theta(i)*( 360/(2*pi) );
       
        if theta(i) > 2*pi*(voltas+1)
       
            voltas=voltas+1;
           
        end
       
    % Cálculo do Fluxo
   
        %determinação da  densidade de fluxo nominal
            for j=1:1:length(theta_r);

                if sin( (polos/2)*(2*pi/360)*( theta_r(j) - theta_deg(i) ) ) > sin( (polos/2)*(2*pi/360)*(entrepolo) )

                    densi_fluxo_nom(j)=1;

                elseif sin( (polos/2)*(2*pi/360)*( theta_r(j) - theta_deg(i) ) )< -sin( (polos/2)*(2*pi/360)*(entrepolo) )

                    densi_fluxo_nom(j)=-1;

                else

                    densi_fluxo_nom(j)=0;

                end

            end

        %Determinação do fluxo em cada polo (SEPARAR OS POLOS)
           
        for j=1:1:3*(angulo_sloth/delta_theta);
     
            Fluxo_a(i)=Fluxo_a(i) + B*( (raio_mot^2) - (raio_int^2) )*densi_fluxo_nom(j)*(2*pi/360)*delta_theta ;        % Fluxos por fase
            Fluxo_b(i)=Fluxo_b(i) + B*( (raio_mot^2) - (raio_int^2) )*densi_fluxo_nom(j + angulo_sloth/delta_theta)*(2*pi/360)*delta_theta;
            Fluxo_c(i)=Fluxo_c(i) + B*( (raio_mot^2) - (raio_int^2) )*densi_fluxo_nom(j + 2*angulo_sloth/delta_theta)*(2*pi/360)*delta_theta;

        end
     
    % Determinação da tensão induzida
   
        Ea(i)=polos*N*( Fluxo_a(i)-Fluxo_a(i-1) )/(2*dt);
        Eb(i)=polos*N*( Fluxo_b(i)-Fluxo_b(i-1) )/(2*dt);
        Ec(i)=polos*N*( Fluxo_c(i)-Fluxo_c(i-1) )/(2*dt);

        Eab(i)=Ea(i)+Eb(i);
        Ecb(i)=Ec(i)+Eb(i);
   
     OndaTri=OndaTri+dt;
   
     counter=counter+1
end



figure(1)
plot(t,theta)
title('Angulo por tempo')
%legend('_')
xlabel('tempo(s)')
ylabel('angulo (rad)')

figure(2)
plot(t,Ea)
title('tensão induzida')
%legend('_')
xlabel('tempo(s)')
ylabel('angulo (rad)')


figure(3)
plot(t,Ia)
title('Corrente A')
%legend('_')
xlabel('tempo(s)')
ylabel('corrente(A)')


figure(5)
plot(t,Te,'g-.')
title('toque no tempo')
%legend('_')
xlabel('tempo(s)')
ylabel('Torque (N*m)')



figure(8)
plot(t,w/(2*pi))
title('velocidade angular por tempo')
%legend('_')
xlabel('tempo(s)')
ylabel('velocidade angular (Hz)')


figure(10)
plot(t,Va,'r',t,Vb,'b',t,Vc,'g')
title('tensões de entrada')
%legend('_')
xlabel('tempo(s)')
ylabel('tensão (volt)')

figure(11)
plot(t,Ea,'r',t,Ia,'b')
title('Tensão induzida e Corrente')
%legend('_')
xlabel('tempo(s)')
ylabel('')

%FILTRAGEM de dados

Te_filtrado=zeros(1,nt);

counter=0;

a=50;
K_const=zeros(1,nt);
for i=2:1:nt;
   
    K_const(i)=Ea(i)/w(i);
   
    if (i)<a;
       
        Te_filtrado(i)=sum(Te(1:i))/i;
       
    else
           
        Te_filtrado(i)=sum(Te(i-(a-1):i))/a;
           
    end
   
    counter=counter+1
   
end

figure(12)
plot(t,Te_filtrado,'g-.')
title('toque no tempo filtrado')
legend('_')
xlabel('tempo(s)')
ylabel('Torque (N*m)')

figure(13)
plot(t,Ia,'r',t,-Ib,'b',t,Ic,'g')
title('Corrente A')
legend('_')
xlabel('tempo(s)')
ylabel('corrente(A)')

