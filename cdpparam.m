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
## @deftypefn {} {@var{retval} =} cdpparam (@var{input1}, @var{input2})
##
## @seealso{}
## @end deftypefn

## Author: xidia <xidia@DESKTOP-O241LP9>
## Created: 2024-01-27

function pars = cdpparam(Duration,N)
pars=struct();
%generate attack sequence
%Duration=14;
Nsim=100;
wupb=0.01;
Tc=0.2;
attack=zeros(1,Nsim);
Flag=1;
for i=2:Nsim
  x=rand(1);
  if x > 0.2 && Flag <= Duration
    attack(:, i)=1;
    Flag=Flag+1;
  else
    Flag=1;
  end
end
pars.attack=attack;

%external disturbances
w=NaN(1,Nsim);
for i=1:Nsim
w(:, i)=wupb*sin(2*Tc*(i-1));
end
pars.w=w;

%model parameters
pars.Tc=0.2;
pars.Mc=1.25;
pars.tau=0.1;
pars.hd=0.42;


%simulation parameters
pars.Nsim=Nsim;
%prediction horizon length
pars.N=N;
%maximum attack duration
pars.duration=Duration;
%matrices
pars.Q=[0.1 0; 0 0.1];
pars.R=0.1;
pars.P=[0.1967 0.0734; 0.0734 0.1737];
pars.K=[-0.3169 -1.1566];
pars.rho=0.01;

%initial states and setpoints
pars.x0=[-1.2;-1.0];
pars.xe=[0;0];
pars.ue=[0];

%up bound of external disturbances
pars.wupb=0.01;

%state, input and disturbance scales
pars.Nx=2;
pars.Nu=1;
pars.Nw=1;


%constraints
pars.ulb=-1.5;
pars.uub=1.5;
pars.plb=-2;
pars.pub=2;
pars.vlb=-2;
pars.vub=2;

%constraints constraction coefficients
pars.kesi=0.2;
pars.yita=0.8;


end %function
