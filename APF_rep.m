%Repulsion calculation
function [Frep,Urep,hxpoint,zxpoint,w,N,yingxiang,bbb,zhdian2,dypoint]=APF_rep(X,zaw_x,zaw_y,m,angle_rep,jdup,jddown,hxup,hxdown,point1x,point1y,point2x,point2y,road_x,road_y,maxYX,n,Poo,obsR)
global safeR;
urep=0;Frer=[];Frerx=[];Frery=[];hxpoint=[0,0];zxpoint=[0,0];w=0;
yingxiang=[];zhdian2=[];dypoint=[];%
for i=1:n
    Rrei(i)=(X(1)-zaw_x(i))^2+(X(2)-zaw_y(i))^2; 
    rre(i)=sqrt(Rrei(i)); 
    if rre(i)>Poo(i) 
        Yrerx(i)=0;
        Yrery(i)=0;
        yingxiang(i)=0;
    else 
        Rat(i)=(X(1)-point2x(i))^2+(X(2)-point2y(i))^2;
        rat(i)=sqrt(Rat(i));
        urep=urep+1/2.0*m*(1.0/rre(i)-1.0/Poo(i))^2; 
        Frer(i)=1*m*(1.0/(rre(i)-obsR(i)-safeR)-1.0/Poo(i))*(1.0/(rre(i)^2));
        Frerx(i)=2*(abs(X(1)-zaw_x(i)))*Frer(i)*cos(angle_rep(i));
        Frery(i)=2*(abs(X(2)-zaw_y(i)))*Frer(i)*sin(angle_rep(i));
        yingxiang(i)=1;
    end
end
bbb=0;
Frep(1)=sum(Frerx);
Frep(2)=sum(Frery);
Urep=urep;
N=sum(yingxiang);
N2=sum(maxYX);
if N<N2
    yingxiang=maxYX;
else
end

if (N>0)
    [~,bbb]=min(Rrei);
     xuhao=find(1==yingxiang);
   [~,a1]=size(xuhao);
   for i=1:a1 
       j=xuhao(i);
       dis11=sqrt((jdup(j,1) -road_x).^2+(jdup(j,2)-road_y).^2);
       upe(i)=min(dis11);
       dis22=sqrt((jddown(j,1) -road_x).^2+(jddown(j,2)-road_y).^2);
       downe(i)=min(dis22);
   end
      [maxupe,z1]=max(upe);
      [maxdowne,z2]=max(downe);
      if maxupe<maxdowne
          xuhao11=xuhao(z1);
          hxpoint=hxup(xuhao11,:);
          zxpoint=[point2x((xuhao11)),point2y((xuhao11))];
          dypoint=[point1x((xuhao11)),point1y((xuhao11))];
      else
          xuhao11=xuhao(z2);
          hxpoint=hxdown(xuhao11,:);
          zxpoint=[point2x((xuhao11)),point2y((xuhao11))];
          dypoint=[point1x((xuhao11)),point1y((xuhao11))];
      end
      zhdian2=[((zaw_x(xuhao11)+hxpoint(1)))/2,((zaw_y(xuhao11)+hxpoint(2)))/2];
      WW=sqrt((dypoint(1)-zhdian2(1))^2+(dypoint(2)-zhdian2(2))^2);
      d=sqrt((dypoint(1)-X(1))^2+(dypoint(2)-X(2))^2);
      if d<WW
      a=sqrt((X(1)-hxpoint(1))^2+(X(2)-hxpoint(2))^2);
      b=sqrt((X(1)-zaw_x(xuhao11))^2+(X(2)-zaw_y(xuhao11))^2);
      r=Poo(xuhao11);
      W=(sqrt(1-((a^2+r^2-b^2)/(2*a*r))^2))*a;
      w=abs(W/WW);
      else
      end
else
end
end
