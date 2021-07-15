%--------------projection operator-------------------
function y=proj_saturation(x)

c1=1.4;
c2=1.4;
 

len=length(x);
y=x;
for i=1:len
if x(i)>c1
    y(i)=c1;
elseif x(i)<-c2
    y(i)=-c2;
else  
end
end