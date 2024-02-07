# MPC_DoSattack
This code simulates an acknowledgement-free, robust, resilient MPC of the cart-damper-spring nonlinear system with DoS attacks in the controller-actuator communication channel.
At every sampling time instant, the new MPC framework does not need to know whether the DoS attack happens, and the communication protocol is a UDP-like protocol.
This simulation runs in the Octave platform with the CasADi(https://web.casadi.org/) and mpctools(https://bitbucket.org/rawlings-group/octave-mpctools/downloads/).
We thank the authors who developed these tools.
Functions/scripts:
cdpmodel---------cart-damper-spring model.
cdpparam---------model parameters.
cdpplant---------real cart-damper-spring system with additive disturbances.
rigidMPC---------the simulation of the new MPC formulation.
rigidNMPC--------a function embedded into rigidMPC function.
nominalMPC-------the simulation of the nominal MPC.
main-------------save the data of nominal and new MPC frameworks.
Comparison-------displays the data in figures.   
