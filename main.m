%This is the simulation of the acknowledgement-free resilient MPC for the cart_damper_spring system
clear all
clc

%load parameters of cart_damper_spring system
Duration=13;% attack duration
N=15;%prediction horizon
pars = cdpparam(Duration,N);

%perform simulation
data = rigidMPC(pars);
%perform simulation attack-free case
data1 = nominalMPC(pars);
save parametersduration13 pars;
save dataAttack13 data;
save dataAttackFree13 data1;

%make plot
timebegin=1;
timeend=1;
figure(1)
axis([0 20 -2 2]);
for i=2:(pars.Nsim-1)
  if pars.attack(:,i)<1 && pars.attack(:,i+1)>0
    timebegin=i+1;
  end
  if pars.attack(:,i)>0 && pars.attack(:,i+1)<1
    timeend=i;
    vertices=[timebegin-1,-2; timeend-1,-2; timeend-1, 2; timebegin-1, 2];
    patch(vertices(:,1).*pars.Tc, vertices(:,2), "facealpha",0.1,"edgelighting","flat");
    hold on
    timebegin=1;
    timeend=1;
  else if pars.attack(:,i)>0 && i==pars.Nsim-1
    timeend=pars.Nsim;
    vertices=[timebegin-1,-2; timeend-1,-2; timeend-1, 2; timebegin-1, 2];
    patch(vertices(:,1).*pars.Tc, vertices(:,2), "facealpha",0.1,"edgelighting","flat");
    hold on
    timebegin=1;
    timeend=1;
  end
end
end
legend("DoS");
plot(data.xt, data.x(1,:),'k-', "displayname", "p: resilient", "linewidth", 1.5)
hold on
plot(data.xt, data.x(2,:),'k--',  "displayname", "v: resilient", "linewidth", 1.5)
hold on
plot(data1.xt, data1.x(1,:),'r:',  "displayname", "p: nominal", "linewidth", 1.5)
hold on
plot(data1.xt, data1.x(2,:),'r-.',  "displayname", "v: nominal", "linewidth", 1.5)
hold on
xlabel('time(s)', "fontsize",12)
ylabel('States', "fontsize",12)
legend("location","best","fontsize", 12)


%legend("p", "p: attack-free", "v", "v: attack-free", "location", "northeastoutside");


timebegin=1;
timeend=1;
figure(2)
stairs(data.ut, data.u,'k-', "linewidth", 1.5, "displayname", "u: resilient")
hold on
stairs(data1.ut, data1.u,'r-.',  "linewidth", 1.5, "displayname", "u: nominal")
hold on
legend("location", "best","fontsize", 12)
legend("autoupdate", "off")
xlabel('time(s)',"fontsize", 12)
ylabel('Control input',"fontsize", 12)
axis([0 20 -1.5 1.5]);
hold on
for i=2:(pars.Nsim-1)
  if pars.attack(:,i)<1 && pars.attack(:,i+1)>0
    timebegin=i+1;
  end
  if pars.attack(:,i)>0 && pars.attack(:,i+1)<1
    timeend=i;
    vertices=[timebegin-1,-2; timeend-1,-2; timeend-1, 2; timebegin-1, 2];
    patch(vertices(:,1).*pars.Tc, vertices(:,2), "facealpha",0.1,"edgelighting","flat");
    hold on
    timebegin=1;
    timeend=1;
  else if pars.attack(:,i)>0 && i==pars.Nsim-1
    timeend=pars.Nsim;
    vertices=[timebegin-1,-1.5; timeend-1,-1.5; timeend-1, 1.5; timebegin-1, 1.5];
    patch(vertices(:,1).*pars.Tc, vertices(:,2), "facealpha",0.1,"edgelighting","flat");
    hold on
    timebegin=1;
    timeend=1;
  end
end
end






