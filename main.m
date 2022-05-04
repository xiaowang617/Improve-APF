%% Create path and obstacle environment
tic
run('main_obstacle')
%% Initial parameters of unmanned vehicle
Xo=[0 79.9 0];%Starting position and starting heading angle
k=1;% gain coefficient of gravity
kpatt=4;
m=25;%Gain coefficient of repulsion
V(1)=4.4;
longth0=0.25;%step
longth(1)=0.01;
amax=3;amin=0.1;
jiaospeed=pi/2;
J=600000;
Time=0.05;Timetotal=0;
K=0;
m_Target=[targetx,targety];
Xj=Xo;
maxYX=zeros(1,jtn);
dtmaxYX=zeros(1,dtn);
zjzliang(1)=0;
%% Obstacle environment
m_Obs=[jingtaizaw_x',jingtaizaw_y']; %Static obstacle location
m_ObsR=jingtaizaw_r;                 %Actual radius of static obstacle
safe=0.5;                            %Limit safety distance
m_ObsSafe=m_ObsR+safe;
Po=2*jingtaizaw_r; %Impact range of obstacles
So=2;   %Limit safety distance
%% ***************After initialization, start the main body cycle******************
for j=1:J%循环开始  
    t1=clock;
    Current(j,1)=Xj(1);
    Current(j,2)=Xj(2);
    Current(j,3)=Xj(3);
    lax=Xj(1)+la*cos(Xj(3));lay=Xj(2)+la*sin(Xj(3));
    laxy=[lax,lay,Xj(3)];
   if j<5
       zjzliang(j)=0;
   else
       zjzliang(j)=abs(Current(j,3)-Current(j-1,3));
   zjzliang(j)=(zjzliang(j)+zjzliang(j-1)+zjzliang(j-2)+zjzliang(j-3)+zjzliang(j-4))/5;
   zjzliang(j)=(zjzliang(j)+zjzliang(j-1)+zjzliang(j-2))/3;

   end
    %Real time offset distance calculation
    [e,ela,gfai,angle_att,kk]=distance(Xj,lax,lay,road_x,road_y,road_dy);
    elaj(j)=ela;
    %Call the calculation gravity module to get the gravitational component in X and Y directions
    [Uatt,Fatt0,Ug,Fg0]=PathAtt(elaj,j,k,j,zjzliang);
    Fg(1)=Fg0*cos(gfai);Fg(2)=Fg0*sin(gfai);FG(j)=Fg0;
    Fatt(1)=Fatt0*cos(angle_att);Fatt(2)=Fatt0*sin(angle_att);FATT(j)=Fatt0;
    [angle_rep]=APF_angle(Xj,jingtaizaw_x,jingtaizaw_y,jtn);%Repulsion angle of static obstacle
    [Frep,Urep(j),hxpoint,zxpoint,w,N(j),yingxiang,bbb,zhdian2,dypoint]=APF_rep(Xj,jingtaizaw_x,jingtaizaw_y,m,angle_rep,jdup,jddown,hxup,hxdown,point1x,point1y,point2x,point2y,road_x,road_y,maxYX,jtn,Poo,jingtaizaw_r);%bbb表示受哪个障碍物影响最大

    %N (J), indicating the number of obstacles in the j-th cycle
    if (N(j)>0)&&(j>1)
        if N(j)>N(j-1)
            maxN=N(j);
            maxYX=yingxiang;
        else
            maxN=maxN;
            maxYX=maxYX;
        end
    else
            maxN=0;
            maxYX=zeros(1,jtn);
    end
        
    %Horizontal and vertical sub target points, gravity under weight
    if (abs(Frep(1))>0)
        [ZiFatt22,ZiUatt22]=APF_Att(Xj,zxpoint,k);
        [ZiFatt11,ZiUatt11]=APF_Att(Xj,hxpoint,k);
        ZiFatt33=(1-w).*ZiFatt22+(w).*ZiFatt11;
        if w>0
            Fatt(1)=(1-w)^2*Fatt(1);
            Fatt(2)=(1-w)^2*Fatt(2);
            Fg=(1-w)^2.*(Fg);
        else
        end
    else
        ZiFatt33=[0,0];
    end
    %Dynamic obstacle avoidance module
    if ((Xj(1)>0)&&(Xj(1)<50))
        hao=1;
    else
        if ((Xj(1)>100)&&(Xj(2)<11)&&(Xj(2)<150))
            hao=2;
        else
            hao=0;
            j0=0;
        end
    end
    if hao>0
        dongtaizaw_x(hao)=dongtaizaw_x(hao)+dtzaw_xv(hao)*Time;
        dongtaizaw_y(hao)=dongtaizaw_y(hao)+dtzaw_yv(hao)*Time;
        dtzaw_x(hao,:)=dtzaw_x(hao,:)+dtzaw_xv(hao)*Time;
        dtzaw_y(hao,:)=dtzaw_y(hao,:)+dtzaw_yv(hao)*Time;
        dtzaw_Pox(hao,:)=dtzaw_Pox(hao,:)+dtzaw_xv(hao)*Time;
        dtzaw_Poy(hao,:)=dtzaw_Poy(hao,:)+dtzaw_yv(hao)*Time;
        dtzaw_safex(hao,:)=dtzaw_safex(hao,:)+dtzaw_xv(hao)*Time;
        dtzaw_safey(hao,:)=dtzaw_safey(hao,:)+dtzaw_yv(hao)*Time;
        dtpoint1x(hao)=dtpoint1x(hao)+dtzaw_xv(hao)*Time;
        dtpoint1y(hao)=dtpoint1y(hao)+dtzaw_yv(hao)*Time;
        dtpoint2x(hao)=dtpoint2x(hao)+dtzaw_xv(hao)*Time;
        dtpoint2y(hao)=dtpoint2y(hao)+dtzaw_yv(hao)*Time;
        dthxup(hao,1)=dthxup(hao,1)+dtzaw_xv(hao)*Time;
        dthxup(hao,2)=dthxup(hao,2)+dtzaw_yv(hao)*Time;
        dthxdown(hao,1)=dthxdown(hao,1)+dtzaw_xv(hao)*Time;
        dthxdown(hao,2)=dthxdown(hao,2)+dtzaw_yv(hao)*Time;
        dtjdup(hao,1)=dtjdup(hao,1)+dtzaw_xv(hao)*Time;
        dtjdup(hao,2)=dtjdup(hao,2)+dtzaw_yv(hao)*Time;
        dtjddown(hao,1)=dtjddown(hao,1)+dtzaw_xv(hao)*Time;
        dtjddown(hao,2)=dtjddown(hao,2)+dtzaw_yv(hao)*Time;
        %Calculation of repulsion force of dynamic obstacles
        [dtangle_rep]=APF_angle(Xj,dongtaizaw_x,dongtaizaw_y,dtn);
        [dtFrep,dtUrep(j),dthxpoint,dtzxpoint,dtw,dtN(j),dtyingxiang,dtbbb,dtzhdian2,dtdypoint]=APF_rep(Xj,dongtaizaw_x,dongtaizaw_y,m,dtangle_rep,dtjdup,dtjddown,dthxup,dthxdown,dtpoint1x,dtpoint1y,dtpoint2x,dtpoint2y,road_x,road_y,dtmaxYX,dtn,dtPoo,dongtaizaw_r);
        [Frepv]=APF_Vrep(Xj,dongtaizaw_x,dongtaizaw_y,dtangle_rep,dtn,Poo,V(j),dtzaw_xv,dtzaw_yv,k);
        dtFrep=dtFrep+Frepv;
        %Dynamic obstacles interfere with each other and have the greatest impact on judgment
        if (dtN(j)>0)&&(j>1)
            if dtN(j)>dtN(j-1) 
                dtmaxN=dtN(j);
                dtmaxYX=dtyingxiang;
            else
                dtmaxN=dtmaxN;
                dtmaxYX=dtmaxYX;
            end
        else
            dtmaxN=0;
            dtmaxYX=zeros(1,jtn);
        end
        %Horizontal and vertical sub target points, gravity under weight
        if (abs(dtFrep(1))>0)
            [dtZiFatt22,dtZiUatt22]=APF_Att(Xj,dtzxpoint,k);
            [dtZiFatt11,dtZiUatt11]=APF_Att(Xj,dthxpoint,k);
            dtZiFatt33=(1-dtw).*dtZiFatt22+(dtw).*dtZiFatt11;
            if dtw>0
                Fatt(1)=(1-dtw)^3*Fatt(1);
                Fatt(2)=(1-dtw)^3*Fatt(2);
                Fg=(1-dtw)^3.*(Fg);
            else
            end
        else
            dtZiFatt33=[0,0];
        end 
        %The following is the moving picture part
        j0=j0+1;
        set(h1,'xdata',h1x(hao,:)+j0*dtzaw_xv(hao)*Time);
        set(h1,'ydata',h1y(hao,:)+j0*dtzaw_yv(hao)*Time);
        htux(hao,:)=h1x(hao,:)+j0*dtzaw_xv(hao)*Time;
        htuy(hao,:)=h1y(hao,:)+j0*dtzaw_yv(hao)*Time;
        set(h2,'xdata',h2x(hao,:)+j0*dtzaw_xv(hao)*Time);
        set(h2,'ydata',h2y(hao,:)+j0*dtzaw_yv(hao)*Time);
        set(h3,'xdata',h3x(hao,:)+j0*dtzaw_xv(hao)*Time);
        set(h3,'ydata',h3y(hao,:)+j0*dtzaw_yv(hao)*Time);
        set(h4,'xdata',h4x(hao)+j0*dtzaw_xv(hao)*Time);
        set(h4,'ydata',h4y(hao)+j0*dtzaw_yv(hao)*Time);
        set(h5,'xdata',h5x(hao)+j0*dtzaw_xv(hao)*Time);
        set(h5,'ydata',h5y(hao)+j0*dtzaw_yv(hao)*Time);
        set(h6,'xdata',h6x(hao)+j0*dtzaw_xv(hao)*Time);
        set(h6,'ydata',h6y(hao)+j0*dtzaw_yv(hao)*Time);
        set(h7,'xdata',h7x(hao)+j0*dtzaw_xv(hao)*Time);
        set(h7,'ydata',h7y(hao)+j0*dtzaw_yv(hao)*Time);
        set(h8,'xdata',h8x(hao)+j0*dtzaw_xv(hao)*Time);
        set(h8,'ydata',h8y(hao)+j0*dtzaw_yv(hao)*Time);
        set(h9,'xdata',h9x(hao)+j0*dtzaw_xv(hao)*Time);
        set(h9,'ydata',h9y(hao)+j0*dtzaw_yv(hao)*Time);
        pause(abs(dtzaw_xv(hao)*Time));
    else
        dtZiFatt33=[0,0];dtFrep=[0,0];
    end
   ZiFatt33=ZiFatt33+dtZiFatt33;    %Dynamic and static sub target point gravity summation
   Frep=Frep+dtFrep;%Dynamic and static repulsion summation
   FREP(j)=sqrt((Frep(1))^2+(Frep(2))^2);
    [Ftotal,Utotal,angle(j)]=APF_Ftotal(Fatt,Fg,Frep,ZiFatt33,Uatt,Urep,Ug);
    V(j+1)=2.1*FG(j)/3.6;

        ajia(j)=0;
        if ((V(j+1)-V(j))/Time)>amax
            V(j+1)=V(j)+amax*Time;
        else if ((V(j+1)-V(j))/Time)<-amax
                V(j+1)=V(j)-amax*Time;
            else
            end
        end
        if (abs((V(j+1)-V(j))/Time)<amin)
            V(j+1)=V(j);
        else
        end

longth(j+1)=V(j+1)*Time;
Timetotal(j)=j*Time;
% Front wheel angle increment limit
    if (angle(j)-Xj(3)>=1.5*jiaospeed*Time)
    angle(j)=Xj(3)+1.5*jiaospeed*Time;
    else 
        if (angle(j)-Xj(3)<=-1.5*jiaospeed*Time)
            angle(j)=Xj(3)-1.5*jiaospeed*Time;
        else
            angle(j);
        end
    end           
    if j>1
        Delta(j)=angle(j)-angle(j-1);
    else
    end
    angle_zengliang(j)=angle(j)-Xj(3);
    Xnext(1)=Xj(1)+longth(j)*cos(angle(j));
    Xnext(2)=Xj(2)+longth(j)*sin(angle(j));
    Xnext(3)=angle(j);
    figure (1)
    plot(Xj(1),Xj(2),'r*','markersize',5)
    hold on;
    Xj=Xnext;
    if (Is_Reach(Xj,m_Target,longth(j))==1)
       K=j;
       break;
    end
    t2=clock;
    shijian(j,1)=etime(t2,t1);
end
[VV,Vlast]=size(V);
V(Vlast)=[];
plot(htux(1,:),htuy(1,:),'k');
plot(htux(2,:),htuy(2,:),'k');
set(0,'defaultfigurecolor','w');
xlabel('X axis of position(m)');ylabel('Y axis of position(m)');
figure (2)
plot(Current(:,1),V)
xlabel('distance');ylabel('speed');
figure (3)
plot(Current(:,1),Current(:,3))
xlabel('distance');ylabel('Heading angle');
figure (4)
jj=1:j;
TTime=jj*Time;
plot(Current(:,1),FG);
xlabel('distance');ylabel('Gravitation');
figure (5)
plot(TTime,Current(:,3))
xlabel('Time(s)');ylabel('Heading angle');
figure (6)
plot(TTime,V)
xlabel('Time(s)');ylabel('V(km/h)');
figure (7)
plot(TTime,FG)
xlabel('Time(s)');ylabel('Longitudinal potential field force Fg');
figure (8)
plot(TTime,FATT)
xlabel('Time(s)');ylabel('Transverse potential field force Fatt');
figure (9)
plot(TTime,FREP)
xlabel('Time(s)');ylabel('Repulsive force of obstacles Frep');
figure (9)
plot(TTime,zjzliang)
xlabel('Time(s)');ylabel('Heading angle increment');
toc
disp(['time',num2str(toc)])
disp(['Carried out:',num2str(j),'step'])


