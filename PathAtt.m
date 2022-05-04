%Road gravity calculation
function [Uatt,Fatt0,Ug,Fg0]=PathAtt(ela,j,k,kk,KK)
if j<2
Uatt=k*(ela(j))^2;Fatt0=2*k*(ela(j))^1;
else
    cha=(ela(j)-ela(j-1));
    kd=0;
    Uatt=k*(ela(j))^2;Fatt0=2.5*k*(ela(j))^1+kd*cha;
end
Ug=(1*k)/(KK(kk)+1)^2;Fg0=3*2.75*k/((KK(kk)+1)^2);

end