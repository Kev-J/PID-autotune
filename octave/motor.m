clear all;
clf;

pkg load control;

Ts=0.001;
tmax=1;

Kc=0.1;
J=0.01;
Ke=0.1;
R=0.1;
L=500*10^-6;
f=0;
t = 0:Ts:tmax-Ts;

K= Kc*25
A=Ke*Kc+R*f;
B=L*f+R*J;
C=L*J;

NUM = [K*0.001]
DEN = [C B A]
SYS = tf(NUM, DEN)
SYSF = feedback(SYS);
[y,t] = step(SYSF,t);

% Discretisation %
As=A-(2*B/Ts)+(4*C/(Ts^2))
Bs=2*A-(8*C/(Ts^2))
Cs=A+(2*B/Ts)+(4*C/(Ts^2))

yd = filter([K 2*K K], [Cs Bs As],ones(1,tmax/Ts));

SYSD=c2d(SYS,Ts,'tustin')

hold on;
plot(t,y,'r+');
%plot(t,yd,'bo');
grid;
