function [saida]=PolinomialZ2(Ts,OS,num,den,T)

OSp=OS/100;
disp('overshoot');
disp(OSp);
Real=4/Ts;
disp('real');
disp(Real);
zeta=sqrt((log(OSp))^2/(pi^2+(log(OSp))^2));
disp('zeta');
disp(zeta);
Wn=4/(Ts*zeta);
disp('Wn');
disp(Wn)
Im=(Wn*sqrt(1-zeta^2));
disp('Im');
disp(Im);

r=exp(-zeta*Wn*T);
disp('r');
disp(r);
theta=Wn*T*sqrt(1-zeta^2);
disp('theta');
disp(theta);

z1=r*exp(1i*theta);
disp('z1')
disp(z1)
z2=conj(z1);
disp('z2')
disp(z2)
z3=exp(-Real*10*T);
disp('z3');
disp(z3);
z4=exp(-Real*20*T);
disp(z4)
z5=exp(-Real*30*T);
disp(z5)

D=conv(conv(conv(conv([1 -z1],[1 -z2]),[1 -z3]),[1 -z4]),[1 -z5]);
disp('Raízes de D');
disp(roots(D));

Dinv=fliplr(D)';%empilha os coeficientes
disp('D');
disp(Dinv);
denInv=fliplr(den)';%empilha os coeficientes do denomidador
numInv=fliplr(num)';%troca de ordem os coeficientes

E=[[denInv;0;0] [0;denInv;0] [0;0;denInv] [numInv;0;0] [0;numInv;0] [0;0;numInv]];
disp('Matriz de Sylvester');
disp(E);
M=E\Dinv;
disp('M');
disp(M)

Gz=tf(num,den,T);%função de transferência da planta
Cz=tf([M(6) M(5) M(4)],[M(3) M(2) M(1)],T);%controlador 
Mf=feedback(Gz*Cz,1);%conttrolador em série 
Mf2=feedback(Gz,Cz);%controlador em paralelo
P1=dcgain(Mf)^(-1);%compensação erro de posição
P2=dcgain(Mf2)^(-1);%compensação erro de posição
Gp=tf([0 2.312+1],[1 2.312],T);
Gp2=tf([0 1.619+1],[1 1.619],T);
Gp3=tf([0 0.1607+1],[1 0.1607],T);
Gp4=tf([0 0.1599+1],[1 0.1599],T);
    
Mfp=Mf2*P2;
figure(1)
step(Mf*P1,Mfp);
set(findall(gca,'Type','line'),'LineWidth',2)
legend('Serie','Paralelo');
stepinfo(Mf)
stepinfo(Mfp)
print('Planta2','-depsc')

figure
bode(Mf,'r');%malha fechada série
hold on
bode(Mf2,'b');%malha fechada paralelo
print('Bode21','-depsc')

figure 
Mfd=feedback(Gz,Cz);
disp('Mfd');
disp(zpk(Mfd));
bode(Mfd,'r');
print('Bode22','-depsc')


saida.Mf=zpk(Mf);
saida.Mf2=zpk(Mf2);
saida.Gp=zpk(Gp);
saida.Gp2=zpk(Gp2);
saida.Gp3=zpk(Gp3);
saida.Gp4=zpk(Gp4);
saida.Mfp=zpk(Mfp);
saida.D=D;
saida.C=Cz;
end

