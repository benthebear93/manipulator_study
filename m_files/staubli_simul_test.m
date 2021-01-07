%% set link
import ETS2.*
clc
clear;
syms q1 q2 q3 q4 q5 q6 real
%%%%     J.ang  J.ext(z)  J.off(x)   J.twist(x)   J.rev   displacemnet %%
%%%%         the  D    A  Alp  SIG  OFF     %%%%% 
L(1) = Link([ 0, 0, 0, 0, 0]);
L(2) = Link([ 0, 0, 0, -pi/2, 0]);
L(3) = Link([ 0, 0, 50, 0, 0]); %0.12446 0.4318
L(4) = Link([ 0, 0, 425, -pi/2, 0]); %0.43180 0.02032
L(5) = Link([ 0, 0, 0, pi/2, 0]);
L(6) = Link([ 0, 0, 0, -pi/2, 0]);
%Robot = SerialLink(L, 'name', 'two link');
Robot = SerialLink(L)
Robot.name = 'Staubli';
clkTE = Robot.fkine([-0.523599 0.785398 -0.785398  1.0472 0 0.523599])%[q1 q2 q3 q4 q5 q6]); %, q3 q4 q5 q6
Robot.plot([0, 0, 0, 0, 0, 0])%, 'deg', 'tilesize', 6) %, 0, 0, 0, 0
%% puma test (original)
import ETS2.*
clc
clear;
syms q1 q2 q3 q4 q5 q6 real

l(1) = Link([0, 0, 0, pi/2,0]);
l(2) = Link([0, 0, 0.4318, 0, 0]);
l(3) = Link([0, 0.15, 0.0203, -pi/2]);
l(4) = Link([0, 0.4318, 0, pi/2, 0]);
l(5) = Link([0, 0, 0, -pi/2, 0]);
l(6) = Link([0, 0, 0, 0, 0]);

puma = SerialLink(l)
cIKTE = puma.fkine([0 0 0 0 0 0])
puma.plot([0, 0, 0, 0, 0, 0])

%% staubli test(JJ Craig)
import ETS2.*
clc
clear;
syms q1 q2 q3 q4 q5 q6 real

l(1) = Link([0, 0, 0.05, -pi/2, 0]);
l(2) = Link([0, 0, 0.425, 0, 0]);
l(3) = Link([0, 0.05, 0, pi/2, 0]);
l(4) = Link([0, 0.425, 0, -pi/2, 0]);
l(5) = Link([0, 0, 0, pi/2, 0]);
l(6) = Link([0, 0, 0, 0, 0]);

Robot = SerialLink(l)
Robot.name = 'Staubli';

%ik_sol(joint_coordinate_sol) = Robot.ikine(e-e pose T, joint_coordinate_init, options)
T = transl(0.5, 0.5, 0.5) %SE(3) homogeneous transform 4x4 representing a pure translation x,y and z.
ik_sol = Robot.ikine(T, [0, -pi/2, pi/2, 0, 0, 0])

%cIKTE = Robot.fkine([0 0 0 0 0 0])
Robot.plot(ik_sol)