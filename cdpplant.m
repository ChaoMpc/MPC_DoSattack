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
## @deftypefn {} {@var{retval} =} cdpplant (@var{input1}, @var{input2})
##
## @seealso{}
## @end deftypefn

## Author: xidia <xidia@DESKTOP-O241LP9>
## Created: 2024-01-27

function retval = cdpplant (x,u,w,pars)
  p=x(1);
  v=x(2);
pp=p+pars.Tc*v;
vv=v-pars.Tc*pars.tau/pars.Mc*exp(-p)*p-pars.Tc*pars.hd/pars.Mc*v+pars.Tc*u/pars.Mc+pars.Tc*w/pars.Mc;
retval=[pp; vv];

endfunction
