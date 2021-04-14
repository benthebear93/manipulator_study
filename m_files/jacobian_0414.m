clc
clear
close
format compact
% x, y, z, roll, pitch, yaw (r b a)
%%%%%%%%%% not work %%%%%%%%%%%%%%%%%%%%%%%
% goal_pos = [0.7224 0.04936 0.03671 -126.4286 69.8378 -124.5767];
% goal_pos = [0.520 -0.1139 0.5364 -16.95 59.35 -39.20]
% goal_pos = [0.4342 -0.2311 0.5719 -56.08 41.69 -96.137]
%%%%%%%%%%%%%%% work %%%%%%%%%%%%%%%%%%%%%%

% goal_pos =[0.2944 -0.162 0.2886 -130.813 81.934 -168.438];   % 22670 -> 5221
% goal_pos =[0.6593 -0.4454 0.2513 -130.958 81.921 -168.587]; % 22582 -> 11289
% goal_pos = [0.05 0.05 0.95 0 0 0];                              % 249678 -> 1040
% goal_pos = [0.6488 -0.1931 0.1566 122.5598 54.5653 89.4622];    % 8027 -> 4920
% goal_pos = [0.2944 -0.162 0.2886 -130.81 81.83 -168.438];       % 22048 -> 5015
% goal_pos = [0.638 0.1567 0.2573 -127.5 72.55 -119.286];         % 48413 -> 5258
% goal_pos = [0.56875 -0.27628 0.55934 -0.9157 89.2378 -0.9154]   % 205 -> 100
% goal_pos = [0.56874 0.39345 0.05617 -0.4232 89.2291 -0.4202]    %221 -> 555
% goal_pos = [0.5687 0.04208 0.05617 -0.1211 89.2462 -0.1177]     %209 -> 685
goal_pos = [0.4475 0.05 0.768 0 0.8589 0]                        %2577 -> 963
init_ang =[10 10 10 0 0 0];  
%% DH paramter
theta1=init_ang(1);  d1=0;   a1=0.050;  alpha1=-pi/2; %parameter link1
theta2=init_ang(2);  d2=0;   a2=0.425;  alpha2=0; %parameter link2
theta3=init_ang(3);  d3=0.050;  a3=0;  alpha3=pi/2; %parameter link3
theta4=init_ang(4);  d4=0.425;   a4=0;  alpha4=-pi/2; %parameter link4
theta5=init_ang(5);  d5=0; a5=0;  alpha5=pi/2; %parameter link5
theta6=init_ang(6);  d6=0.1; a6=0;  alpha6=0; %parameter link6
offset2 = -pi/2;
offset3 = pi/2;
%% goal rpy
Xc=goal_pos(1);
Yc=goal_pos(2);
Zc=goal_pos(3);
r = goal_pos(4); % roll
b = goal_pos(5); % pitch
a = goal_pos(6); % yaw;
R31 = -sin(b);
R32 = cos(b)*sin(r);
R21 = sin(a)*cos(b);
% Three rotation matrix compoent for orientation
% 
goal_rpy = [r, b, a];
goal_R = [R31, R32, R21];
disp("goal rpy"); disp(goal_R); disp(goal_rpy);
% Just for dugging

iteration = 0;
end_flag = 0 ;
deltatheta1=0;deltatheta2=0; deltatheta3=0; %theta1,2,3 velocity
deltatheta4=0;deltatheta5=0; deltatheta6=0; %theta4,5,6 velocity
theta = [theta1 theta2 theta3 theta4 theta5 theta6];
while end_flag == 0
    if theta1 >3.14159
        theta1 = -3.14159 + theta1;
    elseif theta1 <-3.14159
        theta1 = -theta1;
    end
    if theta2+offset2 >2.5743606
        theta2 = -2.5743606 + theta2;
    elseif theta2+offset2 <-2.26893
        theta2 = -theta2;
    end
    if theta3 >2.53073
        theta3 = -2.53073 + theta3;
    elseif theta3 <-2.53073
        theta3 = -theta3;
    end
    if theta4 >4.71239
        theta4 = -4.71239 + theta4;
    elseif theta4 <-4.71239
        theta4 = -theta4;
    end
    if theta5 >2.44346
        theta5 = -2.44346 + theta5;
    elseif theta5 <-2.00713
        theta5 = -theta5;
    end
    if theta6 >4.71239
        theta6 = -4.71239 + theta6;
    elseif theta6 <-4.71239
        theta6 = -theta6;
    end
    % Joint limitation
%     result = [theta1 theta2 theta3 theta4 theta5 theta6];
%     disp("result"); disp(result);
    theta1=theta1+1.2*deltatheta1;
    theta2=theta2+1.2*deltatheta2;
    theta3=theta3+1.2*deltatheta3;
    theta4=theta4+1.2*deltatheta4;
    theta5=theta5+1.2*deltatheta5;
    theta6=theta6+1.2*deltatheta6;
%     result = [deg2rad(theta1) deg2rad(theta2) deg2rad(theta3) deg2rad(theta4) deg2rad(theta5) deg2rad(theta6)];
    if(iteration > 500000)
        iteration
        % avoid too many calculation
        end_flag=1;
        iteration = 0;
    end
    T01=[cos(theta1) -cos(alpha1)*sin(theta1) sin(alpha1)*sin(theta1) a1*cos(theta1);
        sin(theta1) cos(alpha1)*cos(theta1)  -sin(alpha1)*cos(theta1) a1*sin(theta1);
        0,sin(alpha1),cos(alpha1),d1;
        0,0,0,1];
    T12=[cos(theta2+offset2) -cos(alpha2)*sin(theta2+offset2) sin(alpha2)*sin(theta2+offset2) a2*cos(theta2+offset2);
        sin(theta2+offset2) cos(alpha2)*cos(theta2+offset2)  -sin(alpha2)*cos(theta2+offset2) a2*sin(theta2+offset2);
        0,sin(alpha2),cos(alpha2),d2;
        0,0,0,1];
    T23=[cos(theta3+offset3) -cos(alpha3)*sin(theta3+offset3) sin(alpha3)*sin(theta3+offset3) a3*cos(theta3+offset3);
        sin(theta3+offset3) cos(alpha3)*cos(theta3+offset3)  -sin(alpha3)*cos(theta3+offset3) a3*sin(theta3+offset3);
        0,sin(alpha3),cos(alpha3),d3;
        0,0,0,1];
    T34=[cos(theta4) -cos(alpha4)*sin(theta4) sin(alpha4)*sin(theta4) a4*cos(theta4);
         sin(theta4) cos(alpha4)*cos(theta4)  -sin(alpha4)*cos(theta4) a4*sin(theta4);
         0,sin(alpha4),cos(alpha4),d4;
         0,0,0,1];
    T45=[cos(theta5) -cos(alpha5)*sin(theta5) sin(alpha5)*sin(theta5) a5*cos(theta5);
         sin(theta5) cos(alpha5)*cos(theta5)  -sin(alpha5)*cos(theta5) a5*sin(theta5);
         0,sin(alpha5),cos(alpha5),d5;
         0,0,0,1];
    T56=[cos(theta6) -cos(alpha6)*sin(theta6) sin(alpha6)*sin(theta6) a6*cos(theta6);
         sin(theta6) cos(alpha6)*cos(theta6)  -sin(alpha6)*cos(theta6) a6*sin(theta6);
         0,sin(alpha6),cos(alpha6),d6;
         0,0,0,1];
%     T67 =[1 0 0 0.1911;
%           0 1 0 0;
%           0 0 1 0.1027;
%           0 0 0 1]; 
    T01;
    T02 = T01*T12;
    T03 = T01*T12*T23;
    T04 = T01*T12*T23*T34;
    T05 = T01*T12*T23*T34*T45;
    T06 = T01*T12*T23*T34*T45*T56;
%     T07 = T06*T67;
    
    R = [T06(1:3,1) T06(1:3,2) T06(1:3,3)]; %Rotation matrix
    
    roll = atan2(R(3,2),R(3,3)); % roll
    %pitch = atan2(-R(3,1),sqrt(R(1,1)^2+R(2,1)^2)); %pitch
    pitch = atan2(-R(3,1),sqrt(R(3,2)*R(3,2)+R(3,3)*R(3,3))); %pitch
    yaw = atan2(R(2,1),R(1,1)); % yaw
%     roll = rad2deg(roll);
%     pitch = rad2deg(pitch);
%     yaw = rad2deg(yaw);
%     
    rpy_deg = [roll,pitch,yaw];
    r = rpy_deg(1); % roll
    b = rpy_deg(2); % pitch
    a = rpy_deg(3); % yaw
    r31 = R(3,1);
    r32 = R(3,2);
    r21 = R(2,1);
    T = [1 0 sin(b);
        0 cos(r) -cos(b)*sin(r);
        0 sin(r) cos(b)*cos(r)];
    
    invT = inv(T);
    P0=[0 0 0]';
    P1=T01(1:3,4);
    P2=T02(1:3,4);
    P3=T03(1:3,4);
    P4=T04(1:3,4);
    P5=T05(1:3,4);
    P6=T06(1:3,4);
%     P7=T07(1:3,4);
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
    
%     Z6  = T06(1:3,3);
%     Jv7 = cross(Z6,(P7-P6));
    
    Jv=[Jv1 Jv2 Jv3 Jv4 Jv5 Jv6;
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
    
    thetadot=pinv(Ja)*[Xspeed;Yspeed;Zspeed;rspeed;pspeed;yawspeed]*0.1; %
    error_x = Xspeed;
    error_y = Yspeed;
    error_z = Zspeed;
    error_r = rspeed;
    error_p = pspeed;
    error_yaw = yawspeed;

    if abs(error_x)<=0.001 && abs(error_y)<=0.001 && abs(error_z)<=0.001 && abs(error_r)<=0.0001 && abs(error_p)<=0.0001 && abs(error_yaw)<=0.0001
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

    deltatheta1=theta1dot;
    deltatheta2=theta2dot;
    deltatheta3=theta3dot;
    deltatheta4=theta4dot;
    deltatheta5=theta5dot;
    deltatheta6=theta6dot;
    iteration = iteration +1;
end
% T06
iteration
result_deg = [theta1 theta2 theta3 theta4 theta5 theta6]
result = [deg2rad(theta1) deg2rad(theta2) deg2rad(theta3) deg2rad(theta4) deg2rad(theta5) deg2rad(theta6)];
q1 = theta1; 
q2 = theta2;
q3 = theta3;
q4 = theta4;
q5 = theta5;
q6 = theta6;
T67 =[1 0 0 0.1911;
      0 1 0 0;
      0 0 1 0.1027;
      0 0 0 1]
T06
T07= T06*T67;
disp("End_EE"); disp(T07);
EE_ROT = [T06(1:3,1) T06(1:3,2) T06(1:3,3)];
end_rpy = r2rpy(EE_ROT);
disp("end_rpy"); disp(end_rpy);

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
rotz = [cos(a) -sin(a) 0;
    sin(a) cos(a) 0;
    0 0 1];

roty = [cos(b) 0 sin(b);
    0 1 0;
    sin(b) 0 cos(b)];

rotx = [1 0 0;
    0 cos(r) -sin(r);
    0 sin(r) cos(r)];
R = rotz*roty*rotx;

end