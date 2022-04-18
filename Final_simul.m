%% 전력전자 Final
clear all; close all; clc;

%% Buck Convertor Modeling
clear all; close all; clc;

Vin=12;
d=5/12; %0.417
Vref= 3.5*d;
Vm=3.5;
H=Vm/12;

L= 220*10^(-6);
C= 100*10^(-6);
R=10;
Vg=12;

num=[Vg/(L*C)];
den=[1 1/(R*C) 1/(L*C)];

Gvd=tf(num,den)
opt = stepDataOptions('StepAmplitude',d);

figure();
step(Gvd,opt);
stepinfo(Gvd)
grid on,

figure();
pzmap(Gvd);
grid on,


%% find Z range by using Root Locus
close all; clc;

z1=500;                      z2=700;                   z3=800;                     z4=1000;
Gc1=tf([1 z1],[1 0]);        Gc2=tf([1 z2],[1 0]);       Gc3=tf([1 z3],[1 0]);        Gc4=tf([1 z4],[1 0]);
G1=Gc1*(1/Vm)*Gvd;     G2=Gc2*(1/Vm)*Gvd;    G3=Gc3*(1/Vm)*Gvd;      G4=Gc4*(1/Vm)*Gvd;

figure(); 
subplot(2,2,1), rlocus(G1)
subplot(2,2,2), rlocus(G2)
subplot(2,2,3), rlocus(G3)
subplot(2,2,4), rlocus(G4)

%%

close all; clc;

z1=500;                      z2=1000;                   z3=1500;                     z4=2000;
Gc1=tf([1 z1],[1 0]);        Gc2=tf([1 z2],[1 0]);       Gc3=tf([1 z3],[1 0]);        Gc4=tf([1 z4],[1 0]);
G1=Gc1*(1/Vm)*Gvd;     G2=Gc2*(1/Vm)*Gvd;    G3=Gc3*(1/Vm)*Gvd;      G4=Gc4*(1/Vm)*Gvd;

figure(); 
subplot(2,2,1), rlocus(G1)
subplot(2,2,2), rlocus(G2)
subplot(2,2,3), rlocus(G3)
subplot(2,2,4), rlocus(G4)

%% Simlation Voltage Controller
close all; clc;

z=800; %ki=K*z 위에서 찾은 f
K=1; %kp값

Gc=tf([1 z],[1 0]);
C=K;
G=Gc*(1/Vm)*Gvd;

%% SISOTOOL
sisotool(G,C,H,Vref) 

%% simulation Voltage Controller 
close all; clc;

K=0.3; %위에서 정한 값
Gcl=feedback(K*G,H);

opt = stepDataOptions('StepAmplitude',Vref);

figure();
bode(Gcl,opt)

figure();
step((1/Vm)*Gvd,opt)

figure();
step(Gcl,opt)
xlim([0 0.04])
grid on,

stepinfo(Gcl)


%% simulation Voltage Controller 

clear all; close all; clc;

% buck input(disturbance)
Vg=12;

% design buck Gvd

d=5/12; %0.417
L= 220*10^(-6);
C= 100*10^(-6);
R=10;

num=[Vg/(L*C)];
den=[1 1/(R*C) 1/(L*C)];

Gvd=tf(num,den);

% design compensator

Vm=3.5;

z=800;
K=0.3;

Gc=tf([1 z],[1 0]);
G=Gc*(1/Vm)*Gvd;

% simulate voltage controller

Vref= Vm*d;
H=Vm/12;

Gcl=feedback(K*G,H)

opt = stepDataOptions('StepAmplitude',Vref);

% figure();
% step((1/Vm)*Gvd,opt)
% 
% figure();
% step(Gcl,opt)
% xlim([0 0.04])
% grid on,

S = stepinfo(Gcl,'SettlingTimeThreshold',0.04,'RiseTimeThreshold',[0.00 0.81])

figure();
pzmap(Gcl)
grid on,


%nyquist(K*Gc*Gvd)