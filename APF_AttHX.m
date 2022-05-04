%Sub target point gravity calculation
function [ZiFatt,Uatt]=APF_AttHX(X,zimubiaox,zimubiaoy,m_Obs,m_ObsR,k,n,Po)
uatt=0;
for i=1:n
    R(i)=(X(1)-zimubiaox(i))^2+(X(2)-zimubiaoy(i))^2;
    r(i)=sqrt(R(i));
    Rrei(i)=(X(1)-m_Obs(i,1))^2+(X(2)-m_Obs(i,2))^2;
    rre(i)=sqrt(Rrei(i))-m_ObsR(i);
    if rre(i)>Po
          ZiFattx(i)=0;
          ZiFatty(i)=0;
    else 
    deltaXi=zimubiaox(i)-X(1);
    deltaYi=zimubiaoy(i)-X(2);
    if deltaXi>0
        theta=atan(deltaYi/deltaXi);
    else
        theta=pi+atan(deltaYi/deltaXi);
    end
    angle=theta;
    ZiFattx(i)=2*k*r(i)*cos(angle);
    ZiFatty(i)=2*k*r(i)*sin(angle);
    uatt=uatt+1/2.0*k*R;
    end
    ZiFatt(1)=sum(ZiFattx);
    ZiFatt(2)=sum(ZiFatty);
    Uatt=uatt;
end