close all; clear;
theta1=0;d1=0;a1=10;alpha1=0; %parameter link1
theta2=0;d2=0;a2=10;alpha2=0; %parameter link2
theta3=0;d3=0;a3=5;alpha3=0; %parameter link3

deltatheta1=0;deltatheta2=0; deltatheta3=0; %theta1,2,3 velocity
Xc=1;Yc=15;
a=0;b=0;
count = 0;
while a==0
    while b==0;
        count =  count +1;
        if count>100
            b=1;
            count = 0;
        end
        if theta1>180
           theta1 =theta1 - 180;
        elseif theta1 <0
            theta1 = -theta1;
            disp("theta1           under -180 : ");disp(theta1);
        end
        if theta2>180
           theta2 =theta2 - 180;
        elseif theta2 <-180
            theta2 = theta1 +180;
            disp("theta2            under -180 : ");disp(theta2);
        end
        if theta3>30
           theta3 =theta3 - 30;
        elseif theta3 <-30
            theta3 = theta3 +30;
            disp("theta3                under -180 : ");disp(theta3);
        end
        disp("========================");
        disp("normal theta1 : ");disp(theta1);
        disp("normal theta2 : ");disp(theta2);
        disp("normal theta3 : ");disp(theta3);
        theta1=theta1+deltatheta1/2;
        theta2=theta2+deltatheta2/2;
        theta3=theta3+deltatheta3/2;%360-theta1-theta2;
        T01=DHmatrix(theta1,d1,a1,alpha1);
        T12=DHmatrix(theta2,d2,a2,alpha2);
        T23=DHmatrix(theta3,d3,a3,alpha3);
        T02=T01*T12;
        T03=T01*T12*T23;
        P0=[0 0];
        P1=transpose(T01(1:2,4));
        P2=transpose(T02(1:2,4));
        P3=transpose(T03(1:2,4));
        Q1=[P0(1,1) P1(1,1) P2(1,1) P3(1,1)];
        Q2=[P0(1,2) P1(1,2) P2(1,2) P3(1,2)];
        %disp('end effector pos:');disp(T03);
        
        plot(Q1,Q2,'-o','LineWidth',4);
        axis([-25,25,-25, 25]);
        grid on;
        
    
        %Linear velocity jacobian
        %Prismatic: Jv = zi-1
        %Revolute: Jv = zi-1 x (on - oi-1)

        Z0=[0;0;1];O=[0;0;0];O3=T03(1:3,4);
        Jv1=cross(Z0,(O3-O));
        Z1=T01(1:3,3);O1=T01(1:3,4);
        Jv2=cross(Z1,(O3-O1));
        Z3=T12(1:3,3);O2=T12(1:3,4);
        Jv3=cross(Z1,(O3-O2));
        Jv=[Jv1 Jv2 Jv3];
        %disp('Jv:'); disp(Jv);
        
        Xinit=P3(1,1);Yinit=P3(1,2);
        Xend=Xc;Yend=Yc; 
        Xspeed=(Xend-Xinit);
        Yspeed=(Yend-Yinit);
        %Pseudoinverse Jacobian
        thetadot=pinv(Jv)*[Xspeed;Yspeed;0];
        OrinEnd=atan2d(Yend,Xend);
        Orininit=atan2d(Yinit,Xinit);
        dis_error=sqrt(Xend^2+Yend^2)- sqrt(Xinit^2+Yinit^2);
        orin_error=OrinEnd-Orininit;
        if abs(dis_error)<=0.2 && abs(orin_error)<=0.2
            b=1;
        end
        theta1dot=thetadot(1,1);
        theta2dot=thetadot(2,1);
        theta3dot=thetadot(3,1);
        %disp('thetadot:');disp(thetadot);
       
        deltatheta1=radtodeg(theta1dot);
        deltatheta2=radtodeg(theta2dot);
        deltatheta3=radtodeg(theta3dot);
        deltatheta=[deltatheta1;deltatheta2;deltatheta3];
        %disp('deltatheta');disp(deltatheta);
        text(P3(1,1),P3(1,2),['  (', num2str(P3(1,1),3), ', ', num2str(P3(1,2),3), ')']);
        text(-25,-17,'Orinerror:','Color','red','FontSize',12)
        text(-25,-20,num2str(abs(orin_error),3),'Color','red','FontSize',12)
        text(-25,-23,'diserror:','Color','red','FontSize',12)
        text(-25,-26,num2str(abs(dis_error),3),'Color','red','FontSize',12)
        pause(0.01);
        end_q = [theta1, theta2, theta3];
        %disp(end_q);
    end
    if b==1
       [Xc,Yc,buttons] = ginput;
       end_q = [theta1, theta2, theta3];
       %disp(end_q);
       b=0;
    end
end
