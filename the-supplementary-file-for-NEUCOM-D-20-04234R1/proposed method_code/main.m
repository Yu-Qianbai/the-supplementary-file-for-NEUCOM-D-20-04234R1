clear all
close all
global ita1;
global ita2;
global L;
global Lambda;
global n;
global c0;
global c1;
global c2;
global w
global d1 d4 d5 d6 a2 a3
%-------some parameters of the manipulator---------
d1=89.459/1000;
d4=109.15/1000;
d5=94.65/1000;
d6=82.3/1000;
a2=-425/1000;
a3=-392.25/1000;
c0=50000; %1/\alpha = 50000, \alpha = 0.000002
ts=10;
w=0.64;
c1=1;
c2=1000; %1/\kappa = 1000, \kappa = 0.001
dt=0.1;
t_store=0:dt:ts;
n=10; %number of manipulators

original_set_jla=diag(ones(n,1));
set_jla=[original_set_jla(2:n, :); original_set_jla(1, :)];
jla = set_jla; %adjacent matrix
    
L=diag(jla*ones(n,1))-jla; %Laplacian matrix
Lambda=zeros(n,n);
Lambda(1,1)=1;
Lambda(6,6)=1; %status flag \rho
myoption = odeset('MaxStep',0.01); 
y0=[2.38346661585028,-0.151225134703923,-1.15794685514094,0.795058501385104,-2.89267206649013,0.705500589519546,2.38798362157087,-1.17197112882234,1.03504289672880,0.198978172360145,-3.17655212015867,0.696033199480432,2.38717010486150,-1.12313298601144,0.969156634570870,0.281173145837148,-3.03152824259776,0.730227613533363,2.38604676876570,-1.11586302845424,0.958626975780183,0.264725652432828,-2.97668185356119,0.616600054519396,2.38026970722382,-0.141051603339653,-1.19386206892020,0.667727305615276,-2.81703453836895,0.691507641552159,2.38749647077556,-1.13053496109983,0.978979306574294,0.268976706193471,-3.05363294391952,0.762385173526468,2.38362081612219,-0.200515195297269,-0.963935367014987,1.17591314256513,-2.89688834594718,0.653318831516949,2.36961253817496,-1.00384214707975,0.818042219387380,0.459418532417823,-2.63943410418256,0.758192420490514,2.38450801423210,-0.168384546959594,-1.10033427795563,0.927424211620660,-2.92272449019992,0.669713092493144,2.38526285104105,-0.175529519877509,-1.07249055038678,1.00186024441212,-2.94733937644714,0.667654743509268]';
Y01=[0.0*ones(6*n,1)+y0;zeros(9*n,1)]; %initial input
[TOUT0,YOUT0]= ode15s(@myfunc,[0,ts/2],Y01,myoption);

set_jla=zeros(n,n);
randIndex1 = randperm(size(set_jla, 1));
randIndex = [randIndex1,randIndex1(1)]; 
    for i=1:n
         set_jla(randIndex(i),randIndex(i+1))=1;
    end
jla = set_jla;  
L=diag(jla*ones(n,1))-jla;
Y02=YOUT0(end, :);
[TOUT2,YOUT2]= ode15s(@myfunc,[ts/2,ts],Y02,myoption); %ode solver
YOUT=[YOUT0(1: end-1, :); YOUT2];
TOUT=[TOUT0(1: end-1, :);TOUT2];

u_store=interp1(TOUT,YOUT,t_store);
clear TOUT
clear YOUT
TOUT1=t_store';
YOUT1=u_store;

myvisualization;