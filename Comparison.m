%load parameters, external disturbances and attack sequence

%different DoS attack duration % Prediction Horizon 14
load parameters;
Duration=[5 7 9 11 13];
N=15;
Sizeduration=max(size(Duration));
ControlPerformance_duration=zeros(2,Sizeduration);
data_rigidMPC=struct('x', NaN(pars.Nx, pars.Nsim+1,Sizeduration), 'u', NaN(pars.Nu, pars.Nsim, Sizeduration));
data_nominalMPC=struct('x', NaN(pars.Nx, pars.Nsim+1,Sizeduration), 'u', NaN(pars.Nu, pars.Nsim, Sizeduration));

rigidMPC5=load('dataAttack5');
rigidMPC7=load('dataAttack7');
rigidMPC9=load('dataAttack9');
rigidMPC11=load('dataAttack11');
rigidMPC13=load('dataAttack13');

nominalMPC5=load('dataAttackFree5');
nominalMPC7=load('dataAttackFree7');
nominalMPC9=load('dataAttackFree9');
nominalMPC11=load('dataAttackFree11');
nominalMPC13=load('dataAttackFree13');

data_rigidMPC.x(:,:,1)=rigidMPC5.data.x;
data_rigidMPC.u(:,:,1)=rigidMPC5.data.u;
data_rigidMPC.x(:,:,2)=rigidMPC7.data.x;
data_rigidMPC.u(:,:,2)=rigidMPC7.data.u;
data_rigidMPC.x(:,:,3)=rigidMPC9.data.x;
data_rigidMPC.u(:,:,3)=rigidMPC9.data.u;
data_rigidMPC.x(:,:,4)=rigidMPC11.data.x;
data_rigidMPC.u(:,:,4)=rigidMPC11.data.u;
data_rigidMPC.x(:,:,5)=rigidMPC13.data.x;
data_rigidMPC.u(:,:,5)=rigidMPC13.data.u;

data_nominalMPC.x(:,:,1)=nominalMPC5.data1.x;
data_nominalMPC.u(:,:,1)=nominalMPC5.data1.u;
data_nominalMPC.x(:,:,2)=nominalMPC7.data1.x;
data_nominalMPC.u(:,:,2)=nominalMPC7.data1.u;
data_nominalMPC.x(:,:,3)=nominalMPC9.data1.x;
data_nominalMPC.u(:,:,3)=nominalMPC9.data1.u;
data_nominalMPC.x(:,:,4)=nominalMPC11.data1.x;
data_nominalMPC.u(:,:,4)=nominalMPC11.data1.u;
data_nominalMPC.x(:,:,5)=nominalMPC13.data1.x;
data_nominalMPC.u(:,:,5)=nominalMPC13.data1.u;

for k=1:Sizeduration
  temp1=0;
  temp2=0;
  for i=1:pars.Nsim
    temp1=temp1+data_rigidMPC.x(:,i,k)'*pars.Q*data_rigidMPC.x(:,i,k)+data_rigidMPC.u(:,i,k)'*pars.R*data_rigidMPC.u(:,i,k);
    temp2=temp2+data_nominalMPC.x(:,i,k)'*pars.Q*data_nominalMPC.x(:,i,k)+data_nominalMPC.u(:,i,k)'*pars.R*data_nominalMPC.u(:,i,k);
  endfor
  ControlPerformance_duration(1,k)=temp1;
  ControlPerformance_duration(2,k)=temp2;
end
save ControlPerformance_duration

%different prediction horizon pars.duration=5;
Duration=5;
Horizon=[7, 9, 11, 13, 15];
Sizehorizon=max(size(Horizon));
ControlPerformance_horizon=zeros(1,Sizehorizon);
data2=struct('x', NaN(pars.Nx, pars.Nsim+1,Sizehorizon), 'u', NaN(pars.Nu, pars.Nsim, Sizehorizon));
pars=cdpparam(Duration,Horizon(1));


for i=1:Sizehorizon
  pars.N=Horizon(i);
  result=rigidMPC(pars);
  data2.x(:,:,i)=result.x;
  data2.u(:,:,i)=result.u;
end
for k=1:Sizehorizon
  temp=0;
  for i=1:pars.Nsim
    temp=temp+data2.x(:,i,k)'*pars.Q*data2.x(:,i,k)+data2.u(:,i,k)'*pars.R*data2.u(:,i,k);
  endfor
  ControlPerformance_horizon(k)=temp;
end

save ControlPerformance_horizon

