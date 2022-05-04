function[]=Draw_Model(X0,Current,m_Obs,m_ObsR,m_Target,n)
figure
axis([0 100 0 100]);
axis equal;hold on;
grid;
X=Current(:,1);
Y=Current(:,2);
plot(X,Y,'r','LineWidth',2);
   for i=1:n
    Theta=0:pi/20:2*pi;
    xx =m_Obs(i,1)+cos(Theta)*m_ObsR(i);
    yy= m_Obs(i,2)+sin(Theta)*m_ObsR(i);
plot(xx,yy,'LineWidth',2);  
   end
plot(m_Target(1),m_Target(2),'pR',X0(1),X0(2),'ms');
text(1,3,'Begin','FontSize',10);
text(20,18,'Obstacle I','FontSize',10);
text(50,-18,'Obstacle II','FontSize',10);
text(m_Target(1)-1,m_Target(2)+3,'Target','FontSize',10);
xlabel('X（单位：m）');
ylabel('Y（单位：m）');
hold on;

end

