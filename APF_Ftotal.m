function[Ftotal,Utotal,angle]=APF_Ftotal(Fatt,Fg,Frep,ZiFatt22,Uatt,Urep,Ug)
FX=Frep(1)+Fg(1)+Fatt(1) +ZiFatt22(1);
FY=Frep(2)+Fg(2)+Fatt(2) +ZiFatt22(2);
Utotal=Uatt+Urep+Ug;
if(FX>0)
    angle=atan(FY/FX);
else
    if(FY>0)
    angle=pi+atan(FY/FX);
    else 
        angle=-pi+atan(FY/FX);
    end
end
Ftotal=sqrt(FX^2+FY^2);
end