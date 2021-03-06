clc
clear
close
format compact
goal_pos =[0.6777 0.05 0.5948 0 68.39 0]; % x, y, z, roll, pitch, yaw (r b a)
init_ang =[10 10 20 3 5 5];
Xend = goal_pos(1);
Yend = goal_pos(2);
Zend = goal_pos(3);
Aend = goal_pos(4); % A = yaw
Bend = goal_pos(5); % B = pitch
Rend = goal_pos(6); % R = roll
%% DH paramter
                    %d(z)    a(x)        a(x)
theta1=init_ang(1);  d1=0;   a1=0.050;  alpha1=-90; %parameter link1
theta2=init_ang(2);  d2=0.050;   a2=0.425;  alpha2=0; %parameter link2 % offset -90
theta3=init_ang(3);  d3=0;  a3=0;  alpha3=90; %parameter link3 % offset 90
theta4=init_ang(4);  d4=0.425;   a4=0;  alpha4=-90; %parameter link4
theta5=init_ang(5);  d5=0; a5=0;  alpha5=90; %parameter link5
theta6=init_ang(6);  d6=0.100; a6=0;  alpha6=0; %parameter link6
offset2 = -90;
offset3 = 90;
%% Homogeneous transformation
T01=[cosd(theta1) -cosd(alpha1)*sind(theta1) sind(alpha1)*sind(theta1) a1*cosd(theta1);
    sind(theta1) cosd(alpha1)*cosd(theta1)  -sind(alpha1)*cosd(theta1) a1*sind(theta1);
    0,sind(alpha1),cosd(alpha1),d1;
    0,0,0,1];
T12=[cosd(theta2+offset2) -cosd(alpha2)*sind(theta2+offset2) sind(alpha2)*sind(theta2+offset2) a2*cosd(theta2+offset2);
    sind(theta2+offset2) cosd(alpha2)*cosd(theta2+offset2)  -sind(alpha2)*cosd(theta2+offset2) a2*sind(theta2+offset2);
    0,sind(alpha2),cosd(alpha2),d2;
    0,0,0,1];
T23=[cosd(theta3+offset3) -cosd(alpha3)*sind(theta3+offset3) sind(alpha3)*sind(theta3+offset3) a3*cosd(theta3+offset3);
    sind(theta3+offset3) cosd(alpha3)*cosd(theta3+offset3)  -sind(alpha3)*cosd(theta3+offset3) a3*sind(theta3+offset3);
    0,sind(alpha3),cosd(alpha3),d3;
    0,0,0,1];
T34=[cosd(theta4) -cosd(alpha4)*sind(theta4) sind(alpha4)*sind(theta4) a4*cosd(theta4);
     sind(theta4) cosd(alpha4)*cosd(theta4)  -sind(alpha4)*cosd(theta4) a4*sind(theta4);
     0,sind(alpha4),cosd(alpha4),d4;
     0,0,0,1];
T45=[cosd(theta5) -cosd(alpha5)*sind(theta5) sind(alpha5)*sind(theta5) a5*cosd(theta5);
     sind(theta5) cosd(alpha5)*cosd(theta5)  -sind(alpha5)*cosd(theta5) a5*sind(theta5);
     0,sind(alpha5),cosd(alpha5),d5;
     0,0,0,1];
T56=[cosd(theta6) -cosd(alpha6)*sind(theta6) sind(alpha6)*sind(theta6) a6*cosd(theta6);
     sind(theta6) cosd(alpha6)*cosd(theta6)  -sind(alpha6)*cosd(theta6) a6*sind(theta6);
     0,sind(alpha6),cosd(alpha6),d6;
     0,0,0,1]; 
T01;
T02 = T01*T12;
T03 = T01*T12*T23;
T04 = T01*T12*T23*T34;
T05 = T01*T12*T23*T34*T45;
T06 = T01*T12*T23*T34*T45*T56; % EE
% disp("init ee T"); disp(T06)
P6=T06(1:3,4); % Position
Rinit = [T06(1:3,1) T06(1:3,2) T06(1:3,3)] %Rotation matrix
% rpyinit = r2rpy(Rinit); % RPY
% %% init rpy
% r = rpyinit(1); % roll
% b = rpyinit(2); % pitch
% a = rpyinit(3); % yaw
% R31 = -sind(b);
% R32 = cosd(b)*sind(r);
% R21 = sind(a)*cosd(b);
% rpy_init = [R31, R32, R21];
% disp("init rpy"); disp(rpy_init);
%% goal rpy
r = goal_pos(4); % roll
b = goal_pos(5); % pitch
a = goal_pos(6); % yaw;
% 
R31 = -sind(b);
R32 = cosd(b)*sind(r);
R21 = sind(a)*cosd(b);

goal_rpy = [R31, R32, R21];
disp("goal rpy"); disp(goal_rpy);
% 확인 완료
% DH paramter 

Xc=goal_pos(1);Yc=goal_pos(2);Zc=goal_pos(3);
% r=goal_pos(6);p=goal_pos(5);yaw=goal_pos(4);
% init pos and orientation(Euler)

iteration = 0;
end_flag = 0 ;
deltatheta1=0;deltatheta2=0; deltatheta3=0; %theta1,2,3 velocity
deltatheta4=0;deltatheta5=0; deltatheta6=0; %theta4,5,6 velocity
theta = [theta1 theta2 theta3 theta4 theta5 theta6];
while end_flag == 0
    if theta1 >180
        theta1 = -180 + theta1;
    elseif theta1 <-180
        theta1 = -theta1;
    end
    if theta2+offset2 >147.5
        theta2 = -147.5 + theta2;
    elseif theta2+offset2 <-130
        theta2 = -theta2;
    end
    if theta3 >145
        theta3 = -145 + theta3;
    elseif theta3 <-145
        theta3 = -theta3;
    end
    if theta4 >270
        theta4 = -270 + theta4;
    elseif theta4 <-270
        theta4 = -theta4;
    end
    if theta5 >140
        theta5 = -140 + theta5;
    elseif theta5 <-115
        theta5 = -theta5;
    end
    if theta6 >270
        theta6 = -270 + theta6;
    elseif theta6 <-270
        theta6 = -theta6;
    end
%     result = [theta1 theta2 theta3 theta4 theta5 theta6];
%     disp("result"); disp(result);
    theta1=theta1+deltatheta1/2;
    theta2=theta2+deltatheta2/2;
    theta3=theta3+deltatheta3/2;
    theta4=theta4+deltatheta4/2;
    theta5=theta5+deltatheta5/2;
    theta6=theta6+deltatheta6/2;
%     result = [deg2rad(theta1) deg2rad(theta2) deg2rad(theta3) deg2rad(theta4) deg2rad(theta5) deg2rad(theta6)];
    if(iteration > 100000)
        % avoid too many calculation
        end_flag=1;
        iteration = 0;
    end
    T01=[cosd(theta1) -cosd(alpha1)*sind(theta1) sind(alpha1)*sind(theta1) a1*cosd(theta1);
        sind(theta1) cosd(alpha1)*cosd(theta1)  -sind(alpha1)*cosd(theta1) a1*sind(theta1);
        0,sind(alpha1),cosd(alpha1),d1;
        0,0,0,1];
    T12=[cosd(theta2+offset2) -cosd(alpha2)*sind(theta2+offset2) sind(alpha2)*sind(theta2+offset2) a2*cosd(theta2+offset2);
        sind(theta2+offset2) cosd(alpha2)*cosd(theta2+offset2)  -sind(alpha2)*cosd(theta2+offset2) a2*sind(theta2+offset2);
        0,sind(alpha2),cosd(alpha2),d2;
        0,0,0,1];
    T23=[cosd(theta3+offset3) -cosd(alpha3)*sind(theta3+offset3) sind(alpha3)*sind(theta3+offset3) a3*cosd(theta3+offset3);
        sind(theta3+offset3) cosd(alpha3)*cosd(theta3+offset3)  -sind(alpha3)*cosd(theta3+offset3) a3*sind(theta3+offset3);
        0,sind(alpha3),cosd(alpha3),d3;
        0,0,0,1];
    T34=[cosd(theta4) -cosd(alpha4)*sind(theta4) sind(alpha4)*sind(theta4) a4*cosd(theta4);
         sind(theta4) cosd(alpha4)*cosd(theta4)  -sind(alpha4)*cosd(theta4) a4*sind(theta4);
         0,sind(alpha4),cosd(alpha4),d4;
         0,0,0,1];
    T45=[cosd(theta5) -cosd(alpha5)*sind(theta5) sind(alpha5)*sind(theta5) a5*cosd(theta5);
         sind(theta5) cosd(alpha5)*cosd(theta5)  -sind(alpha5)*cosd(theta5) a5*sind(theta5);
         0,sind(alpha5),cosd(alpha5),d5;
         0,0,0,1];
    T56=[cosd(theta6) -cosd(alpha6)*sind(theta6) sind(alpha6)*sind(theta6) a6*cosd(theta6);
         sind(theta6) cosd(alpha6)*cosd(theta6)  -sind(alpha6)*cosd(theta6) a6*sind(theta6);
         0,sind(alpha6),cosd(alpha6),d6;
         0,0,0,1];
    T01;
    T02 = T01*T12;
    T03 = T01*T12*T23;
    T04 = T01*T12*T23*T34;
    T05 = T01*T12*T23*T34*T45;
    T06 = T01*T12*T23*T34*T45*T56;
    
    R = [T06(1:3,1) T06(1:3,2) T06(1:3,3)]; %Rotation matrix
    rpycurr = r2rpy(R); % RPY
    r = rpycurr(1); % roll
    b = rpycurr(2); % pitch
    a = rpycurr(3); % yaw
    r31 = R(3,1);
    r32 = R(3,2);
    r21 = R(2,1);
    T = [1 0 sind(b);
        0 cosd(r) -cosd(b)*sind(r);
        0 sind(r) cosd(b)*cosd(r)];
    
%     T = [cosd(rpy(1))*cosd(rpy(2)) -sind(rpy(1)) 0;
%         sind(rpy(1))*cosd(rpy(2))  cosd(rpy(1)) 0;
%         -sind(rpy(2)) cosd(rpy(1)) 1];
    invT = inv(T);
    P0=[0 0 0]';
    P1=T01(1:3,4);
    P2=T02(1:3,4);
    P3=T03(1:3,4);
    P4=T04(1:3,4);
    P5=T05(1:3,4);
    P6=T06(1:3,4);
    %%%%%%%%%%%%%%%%%%%%%%%
    Z0  = [0;0;1];
    Jv1 = cross(Z0,(P6-P0));

    Z1  = T01(1:3,3);
    Jv2 = cross(Z1,(P6-P1));

    Z2  = T02(1:3,3);
    Jv3 = cross(Z2,(P6-P2));

    Z3  = T03(1:3,3);
    Jv4 = cross(Z3,(P6-P3));

    Z4  = T04(1:3,3);
    Jv5 = cross(Z4,(P6-P4));

    Z5  = T05(1:3,3);
    Jv6 = cross(Z5,(P6-P5));
    
    Jv=[Jv1 Jv2 Jv3 Jv4 Jv5 Jv6
        Z0   Z1  Z2  Z3  Z4  Z5];
    
    Ja = [eye(3) zeros(3);
        zeros(3) invT]*Jv;
    
    Xinit = P6(1,1); 
    Yinit = P6(2,1);
    Zinit = P6(3,1);
    
    Xspeed   = goal_pos(1)-Xinit;
    Yspeed   = goal_pos(2)-Yinit;
    Zspeed   = goal_pos(3)-Zinit;
    rspeed   = R31 - r31;
    pspeed   = R32 - r32;
    yawspeed = R21 - r21;
    
    thetadot=pinv(Ja)*[Xspeed;Yspeed;Zspeed;rspeed;pspeed;yawspeed]*0.01; %
    error_x = Xspeed;
    error_y = Yspeed;
    error_z = Zspeed;
    error_r = rspeed;
    error_p = pspeed;
    error_yaw = yawspeed;

    if abs(error_x)<=0.001 && abs(error_y)<=0.001 && abs(error_z)<=0.001 && abs(error_r)<=0.001 && abs(error_p)<=0.001 && abs(error_yaw)<=0.001
        disp("=======================");
        disp("==========Arrive=========");
        disp("=======================");
        end_flag=1;
    end
    theta1dot=thetadot(1,1);
    theta2dot=thetadot(2,1);
    theta3dot=thetadot(3,1);
    theta4dot=thetadot(4,1);
    theta5dot=thetadot(5,1);
    theta6dot=thetadot(6,1);

    deltatheta1=rad2deg(theta1dot);
    deltatheta2=rad2deg(theta2dot);
    deltatheta3=rad2deg(theta3dot);
    deltatheta4=rad2deg(theta4dot);
    deltatheta5=rad2deg(theta5dot);
    deltatheta6=rad2deg(theta6dot);
    iteration = iteration +1
end
% T06
result_deg = [theta1 theta2 theta3 theta4 theta5 theta6]
result = [deg2rad(theta1) deg2rad(theta2) deg2rad(theta3) deg2rad(theta4) deg2rad(theta5) deg2rad(theta6)];
q1 = theta1; 
q2 = theta2;
q3 = theta3;
q4 = theta4;
q5 = theta5;
q6 = theta6;
disp("End_EE"); disp(T06);
EE_ROT = [T06(1:3,1) T06(1:3,2) T06(1:3,3)];
end_rpy = r2rpy(EE_ROT);
disp("end_rpy"); disp(end_rpy);
l(1) = Link([0, 0.0, 0.05, -pi/2, 0.0, 0]); %j2
l(2) = Link([0, 0.05, 0.425, 0, 0.0, -pi/2]); %j3
l(3) = Link([0, 0.0, 0.0,  pi/2, 0.0, pi/2]);
l(4) = Link([0, 0.425, 0.0,  -pi/2, 0.0, 0]);

l(5) = Link([0, 0, 0.0,  pi/2, 0.0, 0.0]);
l(6) = Link([0, 0.1, 0.0,  0, 0.0, 0.0]);
staubli = SerialLink(l, 'name', 'staubli_rtb');
staubli.fkine(result);
staubli.plot(result)

function rpy_deg = r2rpy(R)
%
% Get roll, pitch, and yaw [in degree] from a rotation matrix
%  alpha: yaw
%  beta: pitch
%  gamma: roll 
%

r = atan2(R(3,2),R(3,3)); % roll
p = atan2(-R(3,1),sqrt(R(3,2)*R(3,2)+R(3,3)*R(3,3))); %pitch
y = atan2(R(2,1),R(1,1)); % yaw
r = rad2deg(r);
p = rad2deg(p);
y = rad2deg(y);
rpy_deg = [r,p,y];
end

function rpy2r(rpy)
r = rpy(1);
b = rpy(2);
a = rpy(3);
rotz = [cosd(a) -sind(a) 0;
    sind(a) cosd(a) 0;
    0 0 1];

roty = [cosd(b) 0 sind(b);
    0 1 0;
    sind(b) 0 cosd(b)];

rotx = [1 0 0;
    0 cosd(r) -sind(r);
    0 sind(r) cosd(r)];
R = rotz*roty*rotx;

end