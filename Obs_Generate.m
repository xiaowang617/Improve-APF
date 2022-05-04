function[m_Obs,m_Obs_R]=Obs_Generate(Radius,Axis_X,Axis_Y,n)
m_Obs_R=Radius(1)+(Radius(2)-Radius(1))*rand(1,n);
m_Obs(:,1)=Axis_X(1)+(Axis_X(2)-Axis_X(1))*rand(n,1);
m_Obs(:,2)=Axis_Y(1)+(Axis_Y(2)-Axis_Y(1))*rand(n,1);
end