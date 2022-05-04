function[angle_Obs]=APF_angle(X,zaw_x,zaw_y,n)
  for i=1:n
      deltaXi=zaw_x(i)-X(1);
      deltaYi=zaw_y(i)-X(2);
      if deltaXi>0
          theta=atan(deltaYi/deltaXi);
      else
          theta=pi+atan(deltaYi/deltaXi);
      end
      angle_Obs(i)=pi+theta;
  end
end
