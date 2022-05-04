function[]=Draw_Potential(X0,m_Obs,m,m_ObsR,Po,m_Target,k,n,Axis_X,Axis_Y)
figure(5)
[X,Y]=meshgrid(Axis_X(1):0.25:Axis_X(2),Axis_Y(1):0.25:Axis_Y(2));
Rat=(X-m_Target(1)).^2+(Y-m_Target(2)).^2;
Uatt=1/2.0*k*Rat;
 mesh(X,Y,Uatt);
 set(gca,'xdir','reverse') ;
 set(gca,'ydir','reverse') ;
 title ('Distribution of gravitational potential field');
 figure(6)
a=size(X);
Urep = zeros(a(1),a(2));
  for i=1:n
      Rrei=(X-m_Obs(i,1)).^2+(Y-m_Obs(i,2)).^2;
      rre=sqrt(Rrei)-m_ObsR(i);
      temp1=0.5*m*(1./(rre)-1/Po).^2.*Rat;
  Urep=Urep+temp1;
  end
 mesh(X,Y,Urep);
  set(gca,'xdir','reverse') ;
 set(gca,'ydir','reverse') ;
 title ('Repulsive potential field distribution');
end

