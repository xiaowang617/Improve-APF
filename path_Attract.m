%Path gravity calculation
function [Fpatt]=path_Attract(X,m,m_Obs)

Rat=(X(2)-0)^2;
if m_Obs(1,1)<=X(1)-3
    if  m_Obs(2,1)<=X(1)-3
        Fpatt=m*Rat;
    else
        Fpatt=0;
    end
else
    Fpatt=0;
end

 

end