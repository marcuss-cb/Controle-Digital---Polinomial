function [saida]=PolinomialZ3(Ts,OS,num,den,T)

OSp=OS/100;
disp('overshoot');
disp(OSp);
Real=4/Ts;
disp('real');
disp(Real);
if OS==0
    zeta=1;
else
    zeta=sqrt((log(OSp))^2/(pi^2+(log(OSp))^2));
end
disp('zeta');
disp(zeta);
Wn=4/(Ts*zeta);
disp('Wn');
disp(Wn)
Im=(Wn*sqrt(1-zeta^2));
disp('Im');
disp(Im);

r=exp(-Real*T);
disp('r');
disp(r);
theta=Im*T;
disp('theta');
disp(theta);

z1=r*exp(1i*theta);
z2=conj(z1);
z3=exp(-Real*10*T);
disp('z3');
disp(z3);

D=conv(conv([1 -z1],[1 -z2]),[1 -z3]);
disp('Raízes de D');
disp(roots(D));

Dinv=fliplr(D)';%empilha os coeficientes
disp('D');
disp(Dinv);
denInv=fliplr(den)';%empilha os coeficientes do denomidador
numInv=fliplr(num)';%troca de ordem os coeficientes

E=[[denInv;0] [0;denInv] [numInv;0] [0;numInv]];
disp('Matriz de Sylvester');
disp(E);
M=E\Dinv;
disp('M');
disp(M)

Gz=tf(num,den,T);%função de transferência da planta
Cz=tf([M(4) M(3)],[M(2) M(1)],T);%controlador 
Mf=feedback(Gz*Cz,1);%conttrolador em série 
Mf2=feedback(Gz,Cz);%controlador em paralelo
P1=dcgain(Mf)^(-1);%compensação erro de posição
P2=dcgain(Mf2)^(-1);%compensação erro de posição
    
Mfp=Mf2*P2;
figure(1)
step(Mfp,'r');
set(findall(gca,'Type','line'),'LineWidth',2)
legend('Paralelo');
stepinfo(Mf)
stepinfo(Mf2)
print('Planta3','-depsc')

figure
bode(Mf,'b');%malha fechada série
hold on
bode(Mf2,'r');%malha fechada paralelo
print('Bode31','-depsc')

figure 
Mfd=feedback(Gz,Cz);
disp('Mfd');
disp(zpk(Mfd));
bode(Mfd,'r');
print('Bode32','-depsc')

saida.Mf=zpk(Mf);
saida.Mf2=zpk(Mf2);
saida.Mfp=zpk(Mfp);
saida.D=D;
saida.C=Cz;
end
