function [ZiFatt,Uatt]=APF_Att(X,zipoint,k)
uatt=0;
if (zipoint(1)==0)&&(zipoint(2)==0)
    ZiFatt(1)=0;
    ZiFatt(2)=0;
else 
    deltaX=zipoint(1)-X(1);
    deltaY=zipoint(2)-X(2);
    if deltaX>0
        theta=atan(deltaY/deltaX);
    else
        theta=pi+atan(deltaY/deltaX);
    end
    angle=theta;
    r=sqrt((deltaX)^2+(deltaY)^2);
    ZiFattx=k*r*cos(angle);
    ZiFatty=k*r*sin(angle);
    uatt=uatt+1/2*k*r^2;
end
ZiFatt(1)=(ZiFattx);
ZiFatt(2)=(ZiFatty);
Uatt=uatt;
end
    
