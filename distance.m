function[e,ela,gfai,angle_att,kk]=distance(Xj,lax,lay,road_x,road_y,road_dy)
dis=[];
dis=sqrt((Xj(1) -road_x).^2+(Xj(2)-road_y).^2);
e=min(dis); 
k=find(e==dis);gfai=atan(road_dy(k));
kk=k;
xx=(Xj(1)-road_x(k));
yy=(Xj(2)-road_y(k));
xielv=yy/xx;
jiaodu=atan(xielv);
dis2=[];
dis2=sqrt((lax-road_x).^2+(lay-road_y).^2);
ela=min(dis2);
% Angle of road gravity
if (yy>0)&&(xx>0)                  
    angle_att=jiaodu-pi;
else
    if (yy>0)&&(xx<0)
        angle_att=jiaodu;
    else
        if (yy<0)&&(xx>0)
             angle_att=jiaodu-pi;
        else % yy<0&&xx<0
            angle_att=jiaodu;
        end
    end
end
        
end

