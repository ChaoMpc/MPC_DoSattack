## Copyright (C) 2024 xidia
##
## This program is free software: you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program.  If not, see <https://www.gnu.org/licenses/>.

## -*- texinfo -*-
## @deftypefn {} {@var{retval} =} nominalMPC (@var{input1}, @var{input2})
##
## @seealso{}
## @end deftypefn

## Author: xidia <xidia@DESKTOP-O241LP9>
## Created: 2024-01-28

## Copyright (C) 2024 xidia
##
## This program is free software: you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program.  If not, see <https://www.gnu.org/licenses/>.

## -*- texinfo -*-
## @deftypefn {} {@var{retval} =} rigidMPC (@var{input1}, @var{input2})
##
## @seealso{}
## @end deftypefn

## Author: xidia <xidia@DESKTOP-O241LP9>
## Created: 2024-01-28

function data = nominalMPC (pars)

 mpc = import_mpctools();

 %Read parameters
 Nx=pars.Nx;
 Nu=pars.Nu;
 Nw=pars.Nw;
 N=pars.N;
 Nsim=pars.Nsim;
 Duration=pars.duration;
 x0=pars.x0;
 xss=pars.xe;
 uss=pars.ue;
 wupb=pars.wupb;

%Make functions.
cdp_model=@(x,u) cdpmodel(x,u,pars);
cdp_casadi=mpc.getCasadiFunc(cdp_model, [Nx, Nu], {'x', 'u'}, {'cdp_model'});

%Cost function.
Q=pars.Q;
R=pars.R;
P=pars.P;
l=mpc.getCasadiFunc(@(x, u, xss, uss) (x-xss)'*Q*(x-xss)...
                                      + (u-uss)'*R*(u-uss), [Nx, Nu, Nx, Nu], ...
                                      {'x', 'u', 'xss', 'uss'}, {'l'});
Vf=mpc.getCasadiFunc(@(x, xss) (x-xss)'*P*(x-xss), [Nx, Nx], ...
                                      {'x', 'xss'}, {'Vf'});

%constraints coefficients
ulb=pars.ulb;
uub=pars.uub;
plb=pars.plb;
pub=pars.pub;
vlb=pars.vlb;
vub=pars.vub;
kesi=pars.kesi;
yita=pars.yita;



%terminal constraints
rho=pars.rho; %level set for terminal region
ef=mpc.getCasadiFunc(@(x, xss) (x-xss)'*P*(x-xss)-yita^2*rho, ...
                                   [Nx, Nx], {'x', 'xss'}, {'ef'});


%time-varying trajectory constraints
xlb=ones(2,N+1);
xub=ones(2,N+1);

for i=1:N+1
xlb(:,i)=[plb;vlb]*(1-(i-1)*kesi/N);
xub(:,i)=[pub;vub]*(1-(i-1)*kesi/N);
end

Ulb=ulb*ones(Nu, N);
Uub=uub*ones(Nu,N);

lb=struct('x',xlb,'u',Ulb);
ub=struct('x',xub,'u',Uub);

%make NMPC controller
Ncontrol=struct('x', Nx, 'u', Nu, 't', N);
par=struct('xss', xss, 'uss', uss);

values.xsp=repmat(xss, 1, N+1);
values.usp=repmat(uss,1,N);
guess.x=[linspace(x0(1), xss(1), N+1); linspace(x0(2), xss(2), N+1)];
guess.u=linspace(ulb,uss,N);

%initial NMPC controller
InitialNMPC=mpc.nmpc('f', cdp_casadi, 'x0', x0, 'guess', guess, 'l', l, ...
                     'Vf', Vf, 'ef', ef, 'N', Ncontrol, ...
                     'lb', lb, 'ub', ub, 'par', par, 'verbosity', 0);

%closed-loop simulation.
x=NaN(Nx,Nsim+1);
x(:,1)=x0;
u=NaN(Nu, Nsim);
utemp=NaN(Nu, N);

for i=1:Nsim
%Control loop
       InitialNMPC.fixvar('x', 1, x(:,i));
       InitialNMPC.solve();
       %print status.
       fprintf('%d: %s\n', i, InitialNMPC.status);
           if ~isequal(InitialNMPC.status, 'Solve_Succeeded')
              warning ('Failed at time %d!', i);
              break
          end
       utemp=InitialNMPC.var.u;
%Inject control and simulate plant
if pars.attack(:,i)<1
  actuator=utemp(:,1);
end
u(:,i)=actuator;
x(:, i+1) = cdpplant(x(:, i), u(:,i), pars.w(:,i), pars);
end

xt=(0:Nsim).*pars.Tc;
ut=(0:(Nsim-1)).*pars.Tc;

%output
data=struct ();

data.x = x;
data.xt=xt;
data.u = u;
data.ut=ut;

end%function
