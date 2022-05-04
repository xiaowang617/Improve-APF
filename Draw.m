function[]=Draw(Uatt,Urep,Urept)
 K1=length(Uatt);
  K2=length(Urep);
   K3=length(Urept);
figure(1)
aa=1:K1;
plot(aa,Uatt);
xlabel('Number of steps')
ylabel('gravitational potential field value');
grid on;
title ('Gravitational potential field curve');
figure(2)
bb=1:K2;
plot(bb,Urep);
xlabel('Number of steps')
ylabel('Repulsive potential field value');
grid on;
title ('Repulsive potential field curve');
figure(4)
bb=1:K1;
Uall=Uatt+Urep;
plot(bb,Uall);
xlabel('Number of steps')
ylabel('Total potential field value');
grid on;
title ('Total potential field curve');
end

