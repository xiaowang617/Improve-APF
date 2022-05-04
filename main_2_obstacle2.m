run('main_2_road')
%% Dimensional parameters of vehicle
global r_vehicle;
r_vehicle=2;
%% Static obstacle
jingtaizaw_x=[30];
jingtaizaw_y=[19.9];
jingtaizaw_r=[3];
jtn=max(size(jingtaizaw_r));
theta=0:0.001:2*pi;
safe=0.2;
global safeR;
safeR=safe+r_vehicle;
Po=3*jingtaizaw_r;
Poo=Po+safeR;
point1x=[18.8];
point1y=[20];
point2x=[41.2];
point2y=[20];
hxpoint=[];hxup=[];hxdown=[];
for i=1:jtn
    zaw_x(i,:)=jingtaizaw_x(i)+jingtaizaw_r(i)*cos(theta);
    zaw_y(i,:)=jingtaizaw_y(i)+jingtaizaw_r(i)*sin(theta);
    zaw_Pox(i,:)=jingtaizaw_x(i)+(Poo(i))*cos(theta);
    zaw_Poy(i,:)=jingtaizaw_y(i)+(Poo(i))*sin(theta);
    zaw_safex(i,:)=jingtaizaw_x(i)+(safeR+jingtaizaw_r(i))*cos(theta);
    zaw_safey(i,:)=jingtaizaw_y(i)+(safeR+jingtaizaw_r(i))*sin(theta);
    plot(zaw_x(i,:), zaw_y(i,:),'k','LineWidth',1)
    fill(zaw_x(i,:), zaw_y(i,:),'k')
    plot(zaw_Pox(i,:), zaw_Poy(i,:),'r--','LineWidth',1)
    plot(zaw_safex(i,:), zaw_safey(i,:),'c-.')
    syms xx01 yy01;
    k1(i)=(jingtaizaw_y(i)-point1y(i))/(jingtaizaw_x(i)-point1x(i));k2(i)=-1/k1(i);
    zhdx(i)=(jingtaizaw_x(i)+point1x(i))/2;zhdy(i)=(jingtaizaw_y(i)+point1y(i))/2;
    eq1=(xx01-jingtaizaw_x(i))^2+(yy01-jingtaizaw_y(i))^2-Poo(i)^2;
    eq2=k2(i)*(xx01-zhdx(i))+zhdy(i)-yy01;
    [xx4,yy4]=solve(eq1,eq2);
    xx4=double(xx4);yy4=double(yy4);
    hx1=(abs(xx4));hy1=(abs(yy4));
    hxup(i,:)=[24.4,29.7];
    hxdown(i,:)=[24.4,10.3];
    dis12=sqrt((jingtaizaw_x(i) -road_x).^2+(jingtaizaw_y(i)-road_y).^2);
    e1=min(dis12); 
    k=find(e1==dis12);  
    k3(i)=(jingtaizaw_y(i)-road_y(k))/(jingtaizaw_x(i)-road_x(k));
    syms xx02 yy02;
    eq1=(xx02-jingtaizaw_x(i))^2+(yy02-jingtaizaw_y(i))^2-Poo(i)^2;
    eq2=k3(i)*(xx02-jingtaizaw_x(i))+jingtaizaw_y(i)-yy02;
    [xx5,yy5]=solve(eq1,eq2);
    xx5=double(xx5);yy5=double(yy5);
    jx1=(abs(xx5));jy1=(abs(yy5));
        jdup(i,:)=[jingtaizaw_x(i),jingtaizaw_y(i)+Poo(i)];
        jddown(i,:)=[jingtaizaw_x(i),jingtaizaw_y(i)-Poo(i)];
end
%% Dynamic obstacle
dongtaizaw_x=[1000];
dongtaizaw_y=[1000];
dongtaizaw_r=[1];
dongtaizaw_v=[1];
v_angle=[pi,0];
dtn=max(size(dongtaizaw_r));
dtsafeR=safeR;
dtPo=3*dongtaizaw_r;         
dtPoo=dtPo+dtsafeR;
dtpoint1x=[1000];
dtpoint1y=[1000];
h4x=dtpoint1x;      h4y=dtpoint1y;
dtpoint2x=[1000];
dtpoint2y=[1000];
h5x=dtpoint2x;      h5y=dtpoint2y;
dthxpoint=[];dthxup=[];dthxdown=[];
for j=1:dtn
   dtzaw_x(j,:)=dongtaizaw_x(j)+dongtaizaw_r(j)*cos(theta);
   dtzaw_y(j,:)=dongtaizaw_y(j)+dongtaizaw_r(j)*sin(theta);
   h1x(j,:)=dtzaw_x(j,:);       h1y(j,:)=dtzaw_y(j,:);
   dtzaw_xv(j)=dongtaizaw_v(j)*cos(v_angle(j));
   dtzaw_yv(j)=dongtaizaw_v(j)*sin(v_angle(j));
   dtzaw_Pox(j,:)=dongtaizaw_x(j)+(dtPoo(j))*cos(theta);
   dtzaw_Poy(j,:)=dongtaizaw_y(j)+(dtPoo(j))*sin(theta);
   h2x(j,:)=dtzaw_Pox(j,:);     h2y(j,:)=dtzaw_Poy(j,:);
   dtzaw_safex(j,:)=dongtaizaw_x(j)+(dtsafeR+dongtaizaw_r(j))*cos(theta);
   dtzaw_safey(j,:)=dongtaizaw_y(j)+(dtsafeR+dongtaizaw_r(j))*sin(theta);
   h3x(j,:)=dtzaw_safex(j,:);   h3y(j,:)=dtzaw_safey(j,:);
   plot(dtzaw_x(j,:), dtzaw_y(j,:),'g--','LineWidth',1)
   plot(dtzaw_Pox(j,:),dtzaw_Poy(j,:),'r--','LineWidth',1)
   plot(dtzaw_safex(j,:),dtzaw_safey(j,:),'c-.','LineWidth',1)
   syms xx01 yy01;
   k1(j)=(dongtaizaw_y(j)-dtpoint1y(j))/(dongtaizaw_x(j)-dtpoint1x(j));k2(j)=-1/k1(j);
   zhdx(j)=(dongtaizaw_x(j)+dtpoint1x(j))/2;zhdy(j)=(dongtaizaw_y(j)+dtpoint1y(j))/2; 
   eq1=(xx01-dongtaizaw_x(j))^2+(yy01-dongtaizaw_y(j))^2-dtPoo(j)^2;
   eq2=k2(j)*(xx01-zhdx(j))+zhdy(j)-yy01;
   [xx4,yy4]=solve(eq1,eq2);
   xx4=double(xx4);yy4=double(yy4);
   hx1=(abs(xx4));hy1=(abs(yy4));
   dthxup(j,:)=[1000,1000];
   dthxdown(j,:)=[1000,1000];
   h6x(j)=dthxup(j,1);      h6y(j)=dthxup(j,2);
   h7x(j)=dthxdown(j,1);    h7y(j)=dthxdown(j,2);
   dtjdup(j,1)=dongtaizaw_x(j);dtjdup(j,2)=dongtaizaw_y(j)+dtPoo(j);
   dtjddown(j,1)=dongtaizaw_x(j);dtjddown(j,2)=dongtaizaw_y(j)-dtPoo(j);
   h8x(j)=dtjdup(j,1);h8y(j)=dtjdup(j,2);
   h9x(j)=dtjddown(j,1);h9y(j)=dtjddown(j,2);
   h1=line(h1x(j,:),h1y(j,:));set(h1,'Linestyle','-','color','k');
   h2=line(h2x(j,:),h2y(j,:));set(h2,'Linestyle','-.','color','r');
   h3=line(h3x(j,:),h3y(j,:));set(h3,'Linestyle','--','color','c');
   h4=plot(h4x(j),h4y(j),'k*');set(h4);
   h5=plot(h5x(j),h5y(j),'r+');set(h5);
   h6=plot(h6x(j),h6y(j),'bo');set(h6);
   h7=plot(h7x(j),h7y(j),'bs');set(h7);
   h8=plot(h8x(j),h8y(j),'ro');set(h8);
   h9=plot(h9x(j),h9y(j),'rs');set(h9);
end
