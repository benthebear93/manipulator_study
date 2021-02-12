clc 
close all; clear;
q1=0;d1=0;L1=10;alpha1=0
q2=0;d2=0;L2=10;alpha2=0;

d_q1=0;d_q2=0;
Xc=10;Yc=10;
a=0; b=0;
orin_error = 0;
count =0;
while a==0
    while b==0
        count= count+1;
        if(count > 100)
            b=1;
            count =0;
        end
        q1=q1+d_q1/2;
        q2=q2+d_q2/2;
        T01=GetDHTransform(L1,alpha1, d1, q1);
        T12=GetDHTransform(L2,alpha2, d2, q2);
        T02=T01*T12;
        P0=[0 0];
        P1=transpose(T01(1:2,4));
        P2=transpose(T02(1:2,4));
        Q1=[P0(1,1) P1(1,1) P2(1,1)];
        Q2=[P0(1,2) P1(1,2) P2(1,2)];
        
        plot(Q1,Q2,'-o','LineWidth',4);
        axis([-31,31,-31,31]);
        grid on;
        
        Z0=[0;0;1];O=[0;0;0];O2=T02(1:3,4);
        Jv1=cross(Z0,(O2-O));
        Z1=T01(1:3,3);O1=T01(1:3,4);
        Jv2=cross(Z1,(O2-O1));
        Jv=[Jv1 Jv2];
        
        Xinit=P2(1,1);Yinit=P2(1,2);
        Xend=Xc;Yend=Yc; 
        Xspeed=(Xend-Xinit);
        Yspeed=(Yend-Yinit);
        thetadot=pinv(Jv)*[Xspeed;Yspeed;0];
        OrinEnd=atan2d(Yend,Xend);
        Orininit=atan2d(Yinit,Xinit);
        dis_error=sqrt(Xend^2+Yend^2)- sqrt(Xinit^2+Yinit^2);
        orin_error=OrinEnd-Orininit;
        if abs(dis_error)<=0.02 & abs(orin_error)<=2
            disp('final joint value : '); disp(q1); disp(q2);
            b=1;
        end
        theta1dot=thetadot(1,1);
        theta2dot=thetadot(2,1);
       
        d_q1=rad2deg(theta1dot);
        d_q2=rad2deg(theta2dot);
        deltatheta=[d_q1;d_q2];
        
        text(P2(1,1),P2(1,2),['  (', num2str(P2(1,1),3), ', ', num2str(P2(1,2),3), ')']);
        text(-25,-17,'Orinerror:','Color','red','FontSize',12)
        text(-25,-20,num2str(abs(orin_error),3),'Color','red','FontSize',12)
        text(-25,-23,'diserror:','Color','red','FontSize',12)
        text(-25,-26,num2str(abs(dis_error),3),'Color','red','FontSize',12)
        pause(0.01);
    end
    if b==1
       [Xc,Yc,buttons] = ginput;
       b=0;
       disp('final joint value : '); disp(q1); disp(q2);
    end
end
