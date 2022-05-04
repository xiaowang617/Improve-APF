%% Basic road information
road_x1=0:0.01:60;
road_y1=0*road_x1+80;
road_dy1=0*(road_x1);
road_qulv1=0*road_x1;
road_theta1=-pi/2:0.001:0;
road_x2=60+20*cos(-road_theta1);
road_y2=60+20*sin(-road_theta1);
road_dy2=-cot(-road_theta1);
road_qulv2=1/20*road_theta1./road_theta1;
road_y3=-60:0.01:-30;
road_y3=-road_y3;
road_x3=0*road_y3+80;
road_dy3=-1/0*road_x3;
road_qulv3=0*road_x3;
road_theta2=pi:0.001:3*pi/2;
road_x4=100+20*cos(road_theta2);
road_y4=30+20*sin(road_theta2);
road_dy4=-cot(road_theta2);
road_dy4(1)=cot(pi);
road_qulv4=1/20*road_theta2./road_theta2;
road_x5=100:0.01:150;
road_y5=0*road_x5+10;
road_dy5=0*road_x5;
road_qulv5=0*road_x5;
quxian=atan(29/10);
road_theta3=-pi/2:0.001:quxian-pi/2;
road_x6=150+20*cos(road_theta3);
road_y6=30+20*sin(road_theta3);
road_dy6=-cot(road_theta3);
road_qulv6=1/20*road_theta3./road_theta3;
lianjiex=max(road_x6);lianjiey=max(road_y6);
syms t
F(t)=0.0005*(t-lianjiex)^3-0.0731*(t-lianjiex)^2+2.9*(t-lianjiex)+lianjiey;
f(t)=diff(F(t));
df(t)=diff(f(t));
x=lianjiex:0.1:lianjiex+100;
Fx=double(F(x));fx=double(f(x));dfx=double(df(x));
KK=(abs(dfx))./((1+(fx).^2).^(1.5));
road_x=[road_x1,road_x2,road_x3,road_x4,road_x5,road_x6,x];
road_y=[road_y1,road_y2,road_y3,road_y4,road_y5,road_y6,Fx];
road_dy=[road_dy1,road_dy2,road_dy3,road_dy4,road_dy5,road_dy6,fx];
road_qulv=[road_qulv1,road_qulv2,road_qulv3,road_qulv4,road_qulv5,road_qulv6,KK];
[a01,a00]=size(road_x);
for i=1:a00-1
    if (road_x(i+1)-road_x(i)<10^-6)&&(road_y(i+1)-road_y(i)<10^-6)
        road_x(i)=[];
        road_y(i)=[];
        road_dy(i)=[];
        road_qulv(i)=[];
        [a01,a00]=size(road_x);
    else
        [a01,a00]=size(road_x);
    end
    if road_x(i)>200
        break;
    else
    end
end
targetx=max(road_x);targety=max(road_y);
plot(road_x,road_y,'b','LineWidth',1.5)
hold on
plot(0,80,'ks','markersize',8);plot(targetx,targety,'kp','markersize',9);
axis([-10 280 -20 140]);
set(0,'defaultfigurecolor','w')
xlabel('X axis of position(m)');ylabel('Y axis of position(m)');
text(2,83,'Start');text(targetx+2,targety+3,'Target');
