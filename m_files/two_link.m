import ETS2.*
clc
clear;
%% set link
syms q1 q2 real
%%%%     J.ang  J.ext(z)  J.off(x)   J.twist(x)   J.rev   displacemnet %%
%%%%         the  D    A  Alp  SIG  OFF     %%%%% 
L(1) = Link([ 0 0 1 0 0 0], 'standard');
L(2) = Link([ 0 0 1 0 0 0], 'standard');
%Robot = SerialLink(L, 'name', 'two link');
Robot = SerialLink(L);
Robot.name = 'Staubli';
clkTE = Robot.fkine([q1 q2]);
%Robot.plot([0, 0], 'deg', 'tilesize', 2)
ik_sol = Robot.ikine(SE3(0.5,1,0),'mask',[1 1 0 0 0 0]);
Robot.plot(ik_sol, 'tilesize',2)