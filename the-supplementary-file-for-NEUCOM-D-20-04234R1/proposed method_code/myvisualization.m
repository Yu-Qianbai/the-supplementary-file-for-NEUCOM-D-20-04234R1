global w;
global d1 d4 d5 d6 a2 a3
%-----------------Color matrix-----------------
c = [0.8500 0.3250 0.0980;
    0.4660 0.6740 0.1880;
    0.9290 0.6940 0.1250;
    0.4940 0.1840 0.5560
    0.3010 0.7450 0.9330;
    0.6350 0.0780 0.1840;
    0.9804 0.7843 0.8039;
    0 0.4470 0.7410
    0 0.6667 0.5647
    0.5412 0.4196 0.7451];

all_error_epplot = zeros(0,0);
x00= [1 2.5 4 5.5 7];
y00=[1.5 3];
[x001,y001]=meshgrid(x00,y00);

q_index_all=zeros(1,0);
dotq_index_all=zeros(1,0);
ddotq_index_all=zeros(1,0);

for i=1:n  %n=10;
RobotPos=zeros(4,1);
RobotPos(1)=x001(i);
RobotPos(2)=y001(i);
myPostionstore=zeros(3,0);
Pos1store=zeros(4,0);
Pos2store=zeros(4,0);
Pos3store=zeros(4,0);
Pos4store=zeros(4,0);
Pos5store=zeros(4,0);
Pos6store=zeros(4,0);
vstore=zeros(0,3);

thetastore=zeros(1,0);
dthetastore=zeros(1,0);
ddthetastore=zeros(1,0);

for j=1:length(TOUT1)
    theta1=YOUT1(j,(6*(i-1)+1));
    theta2=YOUT1(j,(6*(i-1)+2));
    theta3=YOUT1(j,(6*(i-1)+3));
    theta4=YOUT1(j,(6*(i-1)+4));
    theta5=YOUT1(j,(6*(i-1)+5));
    theta6=YOUT1(j,(6*(i-1)+6));
    
    dtheta1=YOUT1(j,6*n+(6*(i-1)+1));
    dtheta2=YOUT1(j,6*n+(6*(i-1)+2));
    dtheta3=YOUT1(j,6*n+(6*(i-1)+3));
    dtheta4=YOUT1(j,6*n+(6*(i-1)+4));
    dtheta5=YOUT1(j,6*n+(6*(i-1)+5));
    dtheta6=YOUT1(j,6*n+(6*(i-1)+6));
    
%----------------------transfer matrix====-------------------    
T1=...
[cos(theta1), 0,  sin(theta1), 0;...
 sin(theta1), 0, -cos(theta1), 0;...
           0, 1,            0, d1;...
           0, 0,            0, 1];        
T2=...
[ cos(theta2), -sin(theta2), 0, a2*cos(theta2);...
  sin(theta2),  cos(theta2), 0, a2*sin(theta2);...
            0,            0, 1,                       0;...
            0,            0, 0,                       1];
 T3=...
 [cos(theta3),  -sin(theta3), 0,cos(theta3)*a3;...
  sin(theta3),   cos(theta3), 0,sin(theta3)*a3;...
            0,  0,            1,              0;...
            0,  0,            0,              1];
 T4=...
[ cos(theta4), 0,  sin(theta4),         0;...
  sin(theta4), 0, -cos(theta4),         0;...
            0, 1,            0,         d4;...
            0, 0,            0,         1];
  T5=...
[ cos(theta5),  0, -sin(theta5), 0;...
  sin(theta5),  0,  cos(theta5), 0;...
            0, -1,            0, d5;...
            0,  0,            0, 1];
  T6=...
[ cos(theta6), -sin(theta6), 0,    0;...
  sin(theta6),  cos(theta6), 0,    0;...
            0,            0, 1,    d6;...
            0,            0, 0,    1];
 
Basepos=[0;0;0;1];
Pos1=T1*Basepos+RobotPos;
Pos2=T1*T2*Basepos+RobotPos;
Pos3=T1*T2*T3*Basepos+RobotPos;
Pos4=T1*T2*T3*T4*Basepos+RobotPos;
Pos5=T1*T2*T3*T4*T5*Basepos+RobotPos;
Pos6=T1*T2*T3*T4*T5*T6*Basepos+RobotPos;
myPosition=Pos6(1:3);
Pos1store= [Pos1store,Pos1];
Pos2store= [Pos2store,Pos2]; 
Pos3store= [Pos3store,Pos3]; 
Pos4store= [Pos4store,Pos4]; 
Pos5store= [Pos5store,Pos5]; 
Pos6store= [Pos6store,Pos6]; 
myPostionstore= [myPostionstore,myPosition];   
%-----------------------jacobian matrix--------------------------- 
A =...
[ (2183*cos(theta1))/20000 + (823*cos(theta1)*cos(theta5))/10000 + (17*cos(theta2)*sin(theta1))/40 - (1893*cos(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)))/20000 + (1893*sin(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1)))/20000 - (823*sin(theta5)*(cos(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1)) + sin(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2))))/10000 - (1569*sin(theta1)*sin(theta2)*sin(theta3))/4000 + (1569*cos(theta2)*cos(theta3)*sin(theta1))/4000, (17*cos(theta1)*sin(theta2))/40 - (1893*cos(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)))/20000 - (1893*sin(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)))/20000 + (823*sin(theta5)*(cos(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)) - sin(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3))))/10000 + (1569*cos(theta1)*cos(theta2)*sin(theta3))/4000 + (1569*cos(theta1)*cos(theta3)*sin(theta2))/4000, (823*sin(theta5)*(cos(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)) - sin(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3))))/10000 - (1893*sin(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)))/20000 - (1893*cos(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)))/20000 + (1569*cos(theta1)*cos(theta2)*sin(theta3))/4000 + (1569*cos(theta1)*cos(theta3)*sin(theta2))/4000, (823*sin(theta5)*(cos(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)) - sin(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3))))/10000 - (1893*sin(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)))/20000 - (1893*cos(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)))/20000, (823*cos(theta5)*(cos(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + sin(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2))))/10000 - (823*sin(theta1)*sin(theta5))/10000, 0;...
(2183*sin(theta1))/20000 - (17*cos(theta1)*cos(theta2))/40 + (823*cos(theta5)*sin(theta1))/10000 + (1893*cos(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)))/20000 - (1893*sin(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)))/20000 + (823*sin(theta5)*(cos(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + sin(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2))))/10000 + (1569*cos(theta1)*sin(theta2)*sin(theta3))/4000 - (1569*cos(theta1)*cos(theta2)*cos(theta3))/4000, (17*sin(theta1)*sin(theta2))/40 - (1893*cos(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1)))/20000 - (1893*sin(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)))/20000 + (823*sin(theta5)*(cos(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)) - sin(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1))))/10000 + (1569*cos(theta2)*sin(theta1)*sin(theta3))/4000 + (1569*cos(theta3)*sin(theta1)*sin(theta2))/4000, (823*sin(theta5)*(cos(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)) - sin(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1))))/10000 - (1893*sin(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)))/20000 - (1893*cos(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1)))/20000 + (1569*cos(theta2)*sin(theta1)*sin(theta3))/4000 + (1569*cos(theta3)*sin(theta1)*sin(theta2))/4000, (823*sin(theta5)*(cos(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)) - sin(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1))))/10000 - (1893*sin(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)))/20000 - (1893*cos(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1)))/20000, (823*cos(theta1)*sin(theta5))/10000 + (823*cos(theta5)*(cos(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1)) + sin(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2))))/10000, 0;...
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                0,                                                                                                                                     (1569*sin(theta2)*sin(theta3))/4000 - (1569*cos(theta2)*cos(theta3))/4000 - (17*cos(theta2))/40 - (823*sin(theta5)*(cos(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)) - sin(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2))))/10000 + (1893*cos(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)))/20000 + (1893*sin(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)))/20000,                                                                                                                         (1569*sin(theta2)*sin(theta3))/4000 - (1569*cos(theta2)*cos(theta3))/4000 - (823*sin(theta5)*(cos(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)) - sin(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2))))/10000 + (1893*cos(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)))/20000 + (1893*sin(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)))/20000,                                                                                                 (1893*cos(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)))/20000 - (823*sin(theta5)*(cos(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)) - sin(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2))))/10000 + (1893*sin(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)))/20000,                                                                                      -(823*cos(theta5)*(cos(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)) + sin(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3))))/10000, 0];

v_workspace=A*[dtheta1;dtheta2;dtheta3;dtheta4;dtheta5;dtheta6];  
vstore=[vstore,v_workspace];
dotq_index=(([dtheta1;dtheta2;dtheta3;dtheta4;dtheta5;dtheta6])'*[dtheta1;dtheta2;dtheta3;dtheta4;dtheta5;dtheta6])/2;
dthetastore=[dthetastore;dotq_index];
end
 
figure(1), %plot of generated bee curves
hold on,
real = plot(myPostionstore(1,:), myPostionstore(2,:)) ;
t=0:0.01:10;
a=0.11;
x=a*(cos(3*w*t) + cos(w*t)) + 0.2 + x001(i);
y=a*(sin(5*w*t) + sin(w*t)) - 0.35 + y001(i);
desired = plot(x,y,'r--');
set(real,'linewidth',1.3);
set(desired,'linewidth',1);

figure(2),hold on, %plot of speed of end-effectors
subplot(3,1,1),hold on,
plot(TOUT1,vstore(1,:)) 
box on
title('vx'),
subplot(3,1,2),hold on,
plot(TOUT1,vstore(2,:)) 
box on
title('vy')
subplot(3,1,3),hold on,
plot(TOUT1,vstore(3,:)) 
box on
title('vz') 
%-------------------------bee curve------------------------
a=0.11;
pd=[ a*(cos(3*w*TOUT1) + cos(w*TOUT1)) + 0.2+x001(i),  a*(sin(5*w*TOUT1) + sin(w*TOUT1))- 0.35+y001(i),  0.44+0*TOUT1];
pd=transpose(pd);
errorP=pd-myPostionstore;
numberS=num2str(i);
error_i=(errorP(1,:).^2+errorP(2,:).^2+errorP(3,:).^2).^(0.5);
all_error_epplot = [all_error_epplot, error_i'];

figure(3),title('manipulator 1,2,3') %Three-dimensional view of bee curves tracking conducted by ten UR5 robots
hold on,
grid on;
for kk=0:0.05:0.9
 plot3([RobotPos(1),interp1(TOUT1,Pos1store(1,:),kk*ts,'linear'),interp1(TOUT1,Pos2store(1,:),kk*ts,'linear'),interp1(TOUT1,Pos3store(1,:),kk*ts,'linear'),interp1(TOUT1,Pos4store(1,:),kk*ts,'linear'),interp1(TOUT1,Pos5store(1,:),kk*ts,'linear'),interp1(TOUT1,Pos6store(1,:),kk*ts,'linear')],...
       [RobotPos(2),interp1(TOUT1,Pos1store(2,:),kk*ts,'linear'),interp1(TOUT1,Pos2store(2,:),kk*ts,'linear'),interp1(TOUT1,Pos3store(2,:),kk*ts,'linear'),interp1(TOUT1,Pos4store(2,:),kk*ts,'linear'),interp1(TOUT1,Pos5store(2,:),kk*ts,'linear'),interp1(TOUT1,Pos6store(2,:),kk*ts,'linear')],...
       [RobotPos(3),interp1(TOUT1,Pos1store(3,:),kk*ts,'linear'),interp1(TOUT1,Pos2store(3,:),kk*ts,'linear'),interp1(TOUT1,Pos3store(3,:),kk*ts,'linear'),interp1(TOUT1,Pos4store(3,:),kk*ts,'linear'),interp1(TOUT1,Pos5store(3,:),kk*ts,'linear'),interp1(TOUT1,Pos6store(3,:),kk*ts,'linear')]);
 axis equal
end
 myplot=plot3(Pos6store(1,:),Pos6store(2,:),Pos6store(3,:),'r--');  %plot the  red trajectory
 set(myplot,'linewidth',2);
 axis equal
 view(3)
 dotq_index_all=[dotq_index_all,dthetastore];
end

figure(4),clf(4), %plot of tracking error
times1 = plot(TOUT1,all_error_epplot(:,1),'color',c(1,:));
set(times1,'linewidth',2.5,'LineStyle','-'); hold on;
times2 = plot(TOUT1,all_error_epplot(:,2),'color',c(2,:));
set(times2,'linewidth',2.5,'LineStyle','--'); hold on;
times3 = plot(TOUT1,all_error_epplot(:,3),'color',c(3,:));
set(times3,'linewidth',2.5,'LineStyle',':'); hold on;
times4 = plot(TOUT1,all_error_epplot(:,4),'color',c(4,:));
set(times4,'linewidth',2.5,'LineStyle','-.'); hold on;
times5 = plot(TOUT1,all_error_epplot(:,5),'color',c(5,:));
set(times5,'linewidth',1.5,'LineStyle','-'); hold on;
times6 = plot(TOUT1,all_error_epplot(:,6),'color',c(6,:));
set(times6,'linewidth',1.5,'LineStyle','--'); hold on;
times7 = plot(TOUT1,all_error_epplot(:,7),'color',c(7,:));
set(times7,'linewidth',1.5,'LineStyle',':'); hold on;
times8 = plot(TOUT1,all_error_epplot(:,8),'color',c(8,:));
set(times8,'linewidth',1.5,'LineStyle','-.'); hold on;
times9 = plot(TOUT1,all_error_epplot(:,9),'color',c(9,:));
set(times9,'linewidth',1,'LineStyle',':'); hold on;
times10 = plot(TOUT1,all_error_epplot(:,10),'color',c(10,:));
set(times10,'linewidth',1,'LineStyle','-.'); hold on;
legend('M1','M2','M3','M4','M5','M6','M7','M8','M9','M10'); 
title('error')


figure(5),clf(5),  % plot of theta
theta1 = plot(TOUT1,YOUT1(:,1+6*(1-1):6*1),'color',c(1,:));
set(theta1,'linewidth',1.5,'LineStyle','-'); hold on;
theta2 = plot(TOUT1,YOUT1(:,1+6*(2-1):6*2),'color',c(2,:));
set(theta2,'linewidth',1.5,'LineStyle','--'); hold on;
theta3 = plot(TOUT1,YOUT1(:,1+6*(3-1):6*3),'color',c(3,:));
set(theta3,'linewidth',1.5,'LineStyle',':'); hold on;
theta4 = plot(TOUT1,YOUT1(:,1+6*(4-1):6*4),'color',c(4,:));
set(theta4,'linewidth',1.5,'LineStyle','-.'); hold on;
theta5 = plot(TOUT1,YOUT1(:,1+6*(5-1):6*5),'color',c(5,:));
set(theta5,'linewidth',1,'LineStyle','-'); hold on;
theta6 = plot(TOUT1,YOUT1(:,1+6*(6-1):6*6),'color',c(6,:));
set(theta6,'linewidth',1,'LineStyle','--'); hold on;
theta7 = plot(TOUT1,YOUT1(:,1+6*(7-1):6*7),'color',c(7,:));
set(theta7,'linewidth',1,'LineStyle',':'); hold on;
theta8 = plot(TOUT1,YOUT1(:,1+6*(8-1):6*8),'color',c(8,:));
set(theta8,'linewidth',1,'LineStyle','-.'); hold on;
theta9 = plot(TOUT1,YOUT1(:,1+6*(9-1):6*9),'color',c(9,:));
set(theta9,'linewidth',0.8,'LineStyle','-'); hold on;
theta10 = plot(TOUT1,YOUT1(:,1+6*(10-1):6*10),'color',c(10,:));
set(theta10,'linewidth',0.8,'LineStyle','--'); hold on;
legend([theta1(1),theta2(1),theta3(1),theta4(1),theta5(1),theta6(1),theta7(1),theta8(1),theta9(1),theta10(1)],'M1','M2','M3','M4','M5','M6','M7','M8','M9','M10'); 
title('\theta')

figure(6),clf(6),   % plot of joint velocity
dtheta1 = plot(TOUT1,YOUT1(:,6*n+1+6*(1-1):6*n+6*1),'color',c(1,:));
set(dtheta1,'linewidth',1.5,'LineStyle','-'); hold on;
dtheta2 = plot(TOUT1,YOUT1(:,6*n+1+6*(2-1):6*n+6*2),'color',c(2,:));
set(dtheta2,'linewidth',1.5,'LineStyle','--'); hold on;
dtheta3 = plot(TOUT1,YOUT1(:,6*n+1+6*(3-1):6*n+6*3),'color',c(3,:));
set(dtheta3,'linewidth',1.5,'LineStyle',':'); hold on;
dtheta4 = plot(TOUT1,YOUT1(:,6*n+1+6*(4-1):6*n+6*4),'color',c(4,:));
set(dtheta4,'linewidth',1.5,'LineStyle','-.'); hold on;
dtheta5 = plot(TOUT1,YOUT1(:,6*n+1+6*(5-1):6*n+6*5),'color',c(5,:));
set(dtheta5,'linewidth',1,'LineStyle','-'); hold on;
dtheta6 = plot(TOUT1,YOUT1(:,6*n+1+6*(6-1):6*n+6*6),'color',c(6,:));
set(dtheta6,'linewidth',1,'LineStyle','--'); hold on;
dtheta7 = plot(TOUT1,YOUT1(:,6*n+1+6*(7-1):6*n+6*7),'color',c(7,:));
set(dtheta7,'linewidth',1,'LineStyle',':'); hold on;
dtheta8 = plot(TOUT1,YOUT1(:,6*n+1+6*(8-1):6*n+6*8),'color',c(8,:));
set(dtheta8,'linewidth',1,'LineStyle','-.'); hold on;
dtheta9 = plot(TOUT1,YOUT1(:,6*n+1+6*(9-1):6*n+6*9),'color',c(9,:));
set(dtheta9,'linewidth',0.8,'LineStyle','-'); hold on;
dtheta10 = plot(TOUT1,YOUT1(:,6*n+1+6*(10-1):6*n+6*10),'color',c(10,:));
set(dtheta10,'linewidth',0.8,'LineStyle','--'); hold on;
legend([dtheta1(1),dtheta2(1),dtheta3(1),dtheta4(1),dtheta5(1),dtheta6(1),dtheta7(1),dtheta8(1),dtheta9(1),dtheta10(1)],'M1','M2','M3','M4','M5','M6','M7','M8','M9','M10'); 
title('DTheta')

figure(7),clf(7), %MVN index value
times1 = plot(TOUT1,dotq_index_all(:,1),'color',c(1,:));
set(times1,'linewidth',2.5,'LineStyle','-'); hold on;
times2 = plot(TOUT1,dotq_index_all(:,2),'color',c(2,:));
set(times2,'linewidth',2.5,'LineStyle','--'); hold on;
times3 = plot(TOUT1,dotq_index_all(:,3),'color',c(3,:));
set(times3,'linewidth',2.5,'LineStyle',':'); hold on;
times4 = plot(TOUT1,dotq_index_all(:,4),'color',c(4,:));
set(times4,'linewidth',2.5,'LineStyle','-.'); hold on;
times5 = plot(TOUT1,dotq_index_all(:,5),'color',c(5,:));
set(times5,'linewidth',1.5,'LineStyle','-'); hold on;
times6 = plot(TOUT1,dotq_index_all(:,6),'color',c(6,:));
set(times6,'linewidth',1.5,'LineStyle','--'); hold on;
times7 = plot(TOUT1,dotq_index_all(:,7),'color',c(7,:));
set(times7,'linewidth',1.5,'LineStyle',':'); hold on;
times8 = plot(TOUT1,dotq_index_all(:,8),'color',c(8,:));
set(times8,'linewidth',1.5,'LineStyle','-.'); hold on;
times9 = plot(TOUT1,dotq_index_all(:,9),'color',c(9,:));
set(times9,'linewidth',1,'LineStyle','-'); hold on;
times10 = plot(TOUT1,dotq_index_all(:,10),'color',c(10,:));
set(times10,'linewidth',1,'LineStyle','--'); hold on;
legend('M1','M2','M3','M4','M5','M6','M7','M8','M9','M10'); 
average_dotq_indx_all = sum(sum(dotq_index_all))/length(TOUT1)/n;
title('dotq index')




