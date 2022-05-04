function longth=Is_Near(X,m_Target,alpha,Fg)

xx=X(1)-m_Target(1);
yy=X(2)-m_Target(2);
rr=sqrt(xx^2+yy^2);
if (rr<alpha+10)
    longth=1;
else
    longth1=0.2*Fg;
    if (longth1<=1.5)
        longth=1.5;
    else
        longth=longth1;
    end
    
end
