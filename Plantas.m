clc
clear
close all
syms z
% %Planta 1
% num=[0 0 0 0 0 0.6761];
% den=[1 -0.9324 0 0 0 0];
% T=0.7;
% G=0.6761/(z^5-0.9324*z^4);
% iztrans(G)
% OS=0;
% Ts=20;
% [saida]=PolinomialZ1(Ts,OS,num,den,T);


% % Planta 2
% 
% num=[0 0.0775 0.1916 0.02865];
% den=[1 -1.539 1.062 -0.1381];
% G=(0.0775*z^2+0.1916*z+0.02865)/(z^3-1.539*z^2+1.062*z-0.1381);
% simplify(iztrans(G))
% T=0.9;
% OS=10;
% Ts=10;
% [saida]=PolinomialZ2(Ts,OS,num,den,T);

% planta 3

num=[0 0.3509 -0.554];
den=[1 -2.032 1];
T=0.09;
OS=0;
Ts=1;
[saida]=PolinomialZ3(Ts,OS,num,den,T);

