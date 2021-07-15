function dy=myfunc(t,y)
global w;
global c0;
global L;
global Lambda;
global n;
global c1;
global c2;
global d1 d4 d5 d6 a2 a3

myTheta=y(1:6*n); %joint variable
myDTheta=y(6*n+1:12*n); %joint velocity variable
mybeta=y(12*n+1:15*n); %Lagrange multiplier vector

%----------------------------------------bee curve----------------------------------------------
%The task of moving to a point can be accomplished by setting rd to the coordinates of the point
a=0.11; 
rd=[ a*(cos(3*w*t) + cos(w*t))+0.2 ; a*(sin(5*w*t) + sin(w*t)) - 0.35; 0.44];
dotrd=[ -a*(3*w*sin(3*w*t) + w*sin(w*t)); a*(5*w*cos(5*w*t) + w*cos(w*t));0];

myPostionstore=zeros(3,0);
for i=1:n
    theta1=myTheta((6*(i-1)+1));
    theta2=myTheta((6*(i-1)+2));
    theta3=myTheta((6*(i-1)+3));
    theta4=myTheta((6*(i-1)+4));
    theta5=myTheta((6*(i-1)+5));
    theta6=myTheta((6*(i-1)+6));
    
    jlq=myTheta((6*(i-1)+1):(6*(i-1)+6)) ;
    jldq=myDTheta((6*(i-1)+1):(6*(i-1)+6)) ;
    jlqdq=[jlq;jldq];
    dotA=dotj(jlqdq);   
                                                                                                                                                                                                                                                                                                                                                            
%--------------------------------- Jacobian matrix of the manipulator--------------------------------
A =...
[ (2183*cos(theta1))/20000 + (823*cos(theta1)*cos(theta5))/10000 + (17*cos(theta2)*sin(theta1))/40 - (1893*cos(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)))/20000 + (1893*sin(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1)))/20000 - (823*sin(theta5)*(cos(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1)) + sin(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2))))/10000 - (1569*sin(theta1)*sin(theta2)*sin(theta3))/4000 + (1569*cos(theta2)*cos(theta3)*sin(theta1))/4000, (17*cos(theta1)*sin(theta2))/40 - (1893*cos(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)))/20000 - (1893*sin(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)))/20000 + (823*sin(theta5)*(cos(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)) - sin(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3))))/10000 + (1569*cos(theta1)*cos(theta2)*sin(theta3))/4000 + (1569*cos(theta1)*cos(theta3)*sin(theta2))/4000, (823*sin(theta5)*(cos(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)) - sin(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3))))/10000 - (1893*sin(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)))/20000 - (1893*cos(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)))/20000 + (1569*cos(theta1)*cos(theta2)*sin(theta3))/4000 + (1569*cos(theta1)*cos(theta3)*sin(theta2))/4000, (823*sin(theta5)*(cos(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)) - sin(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3))))/10000 - (1893*sin(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)))/20000 - (1893*cos(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)))/20000, (823*cos(theta5)*(cos(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + sin(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2))))/10000 - (823*sin(theta1)*sin(theta5))/10000, 0;...
(2183*sin(theta1))/20000 - (17*cos(theta1)*cos(theta2))/40 + (823*cos(theta5)*sin(theta1))/10000 + (1893*cos(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)))/20000 - (1893*sin(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)))/20000 + (823*sin(theta5)*(cos(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + sin(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2))))/10000 + (1569*cos(theta1)*sin(theta2)*sin(theta3))/4000 - (1569*cos(theta1)*cos(theta2)*cos(theta3))/4000, (17*sin(theta1)*sin(theta2))/40 - (1893*cos(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1)))/20000 - (1893*sin(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)))/20000 + (823*sin(theta5)*(cos(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)) - sin(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1))))/10000 + (1569*cos(theta2)*sin(theta1)*sin(theta3))/4000 + (1569*cos(theta3)*sin(theta1)*sin(theta2))/4000, (823*sin(theta5)*(cos(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)) - sin(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1))))/10000 - (1893*sin(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)))/20000 - (1893*cos(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1)))/20000 + (1569*cos(theta2)*sin(theta1)*sin(theta3))/4000 + (1569*cos(theta3)*sin(theta1)*sin(theta2))/4000, (823*sin(theta5)*(cos(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)) - sin(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1))))/10000 - (1893*sin(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)))/20000 - (1893*cos(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1)))/20000, (823*cos(theta1)*sin(theta5))/10000 + (823*cos(theta5)*(cos(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1)) + sin(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2))))/10000, 0;...
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                0,                                                                                                                                     (1569*sin(theta2)*sin(theta3))/4000 - (1569*cos(theta2)*cos(theta3))/4000 - (17*cos(theta2))/40 - (823*sin(theta5)*(cos(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)) - sin(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2))))/10000 + (1893*cos(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)))/20000 + (1893*sin(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)))/20000,                                                                                                                         (1569*sin(theta2)*sin(theta3))/4000 - (1569*cos(theta2)*cos(theta3))/4000 - (823*sin(theta5)*(cos(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)) - sin(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2))))/10000 + (1893*cos(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)))/20000 + (1893*sin(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)))/20000,                                                                                                 (1893*cos(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)))/20000 - (823*sin(theta5)*(cos(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)) - sin(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2))))/10000 + (1893*sin(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)))/20000,                                                                                      -(823*cos(theta5)*(cos(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)) + sin(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3))))/10000, 0];
%------------------transfer matrix------------------
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
Pos6=T1*T2*T3*T4*T5*T6*Basepos; 
myPosition=Pos6(1:3); %position of the end-effector
myPostionstore=[myPostionstore;myPosition]; %position of each of the end-effectors
myJ(:,:,i)=A;
mydotJ(:,:,i)=dotA;
J((3*(i-1)+1):(3*i),(6*(i-1)+1):(6*i))=myJ(:,:,i);
dotJ((3*(i-1)+1):(3*i),(6*(i-1)+1):(6*i))=mydotJ(:,:,i);
end  

%------------------------error compensation--------------------------
par_d =c1*( kron(L,eye(3))*(J*myDTheta) + kron(Lambda,eye(3))*(J*myDTheta - (kron(ones(n,1),dotrd))));

%-------------------------DNN-assisted solver------------------------
bjl=kron(Lambda,eye(3))*(kron(ones(n,1),dotrd))-kron(L+Lambda,eye(3))*par_d-c2*(kron(L+Lambda,eye(3))*myPostionstore-kron(Lambda,eye(3))*(kron(ones(n,1),rd)));
myDDTheta = (c0)*(-y(6*n+1:12*n) + proj_saturation(-(kron(L+Lambda,eye(3))*J)'*y(12*n+1:15*n)));
dot_mybeta = (c0)*((kron(L+Lambda,eye(3))*J)*y(6*n+1:12*n)-bjl);

%----------------The return value of the subroutine------------------
dy=[myDTheta; myDDTheta; dot_mybeta];

t