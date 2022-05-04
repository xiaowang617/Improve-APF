function[Frepv]=APF_Vrep(X,zaw_x,zaw_y,APF_angle,n,Poo,V,vx,vy,k)
for i=1:n
    Rrei(i)=(X(1)-zaw_x(i))^2+(X(2)-zaw_y(i))^2; 
    rre(i)=sqrt(Rrei(i));
    vorx(i)=vx(i)-V*cos(X(3));
    vory(i)=vy(i)-V*sin(X(3));
    vor(i)=sqrt((vorx(i))^2+(vory(i))^2);
    if vorx(i)>0
        angle=atan((vory(i))/(vorx(i)));
    else
        angle=pi+atan((vory(i))/(vorx(i)));
    end
     v_angle(i)=pi+angle;
     abs_angle(i)=abs(v_angle(i)-APF_angle(i));
     if abs_angle(i)>pi
         abs_angle(i)=abs_angle(i)-pi;
     else
     end
    if rre(i)>Poo(i)
        Frepvx(i)=0;
        Frepvy(i)=0;
    else
        if  ((abs_angle(i))<(pi/2))
            Frepv(i)=3*k*(vor(i))*cos(abs_angle(i));
            Frepvx(i)=Frepv(i)*cos(APF_angle(i));
            Frepvy(i)=Frepv(i)*sin(APF_angle(i));
        else
            Frepvx(i)=0;
            Frepvy(i)=0;
        end
    end
end
Frepv(1)=sum(Frepvx);
Frepv(2)=sum(Frepvy);
end