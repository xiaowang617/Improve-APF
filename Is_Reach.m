function Y=Is_Reach(X,m_Target,alpha)
xx=X(1)-m_Target(1);
yy=X(2)-m_Target(2);
rr=sqrt(xx^2+yy^2);
if (rr<alpha+1)
    Y=1;
else
    Y=0;
end
end

