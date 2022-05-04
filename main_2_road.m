
clear
road_x=0:0.01:80;
road_y=0*road_x+20;
road_dy=0*(road_x);
road_qulv=0*road_x;
targetx=max(road_x);targety=max(road_y);
plot(road_x,road_y,'b','LineWidth',1.5) 
hold on
plot(0,20,'ks','markersize',8);plot(targetx,targety,'kp','markersize',9);
axis([-10 100 -10 50]);
set(0,'defaultfigurecolor','w')
xlabel('X axis of position(m)');ylabel('Y axis of position(m)');
text(1,22,'Start');text(targetx+1,targety+2,'Target');