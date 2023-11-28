clc
clear
close all
% Time---------------------------------------------------------------------
tf=10;
N=999;
Nt=tf/N;
t=0:tf/N:tf;
% Final condition----------------------------------------------------------
kesi_1_des=[-2;3;1.5];
dkesi_1_des=[0;0;0];
dkesi_2_des=[0;0;0];
psi_des=0;
% Control parameters-------------------------------------------------------
KpT=eye(3);
KdT=2*eye(3);
KpO=eye(3);
KdO=0.5*eye(3);
% Parameters---------------------------------------------------------------
L=0.225;                    % distance between motor and CoM of quadrotor (m)
R=0.075;                    % radius of propeller (m)
thickness=0.03;             % thickness of rotating propeller (m)
Ixx=4.856*10^-3;            % kgm^2
Iyy=4.856*10^-3;            % kgm^2
Izz=8.801*10^-3;            % kgm^2
m=0.468;                    % kg
g0=9.81;                    % m/s^2
D=diag([0.25,0.25,0.25]);   % drag matrix kg/s
% Initial condition--------------------------------------------------------
xc_0=0;
yc_0=0;
zc_0=0;
phi_0=0;
theta_0=0;
psi_0=0;
u_0=0;
v_0=0;
w_0=0;
p_0=0;
q_0=0;
r_0=0;
q0=[xc_0;yc_0;zc_0;phi_0;theta_0;psi_0];
dq0=[u_0;v_0;w_0;p_0;q_0;r_0];
x0=[q0;dq0];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i=1:length(t)
    x(:,1)=x0;
    % State definition-----------------------------------------------------
    xc=x(1,i);
    yc=x(2,i);
    zc=x(3,i);
    phi=x(4,i);
    theta=x(5,i);
    psi=x(6,i);
    
    u=x(7,i);
    v=x(8,i);
    w=x(9,i);
    p=x(10,i);
    q=x(11,i);
    r=x(12,i);
    % Kinematics' matrices-------------------------------------------------
    Rb=[cos(theta)*cos(psi),sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi),cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);...
        cos(theta)*sin(psi),sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi),cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);...
        -sin(theta),sin(phi)*cos(theta),cos(phi)*cos(theta)];
    
    T=[1,sin(phi)*tan(theta),cos(phi)*tan(theta);...
        0,cos(phi),-sin(phi);...
        0,sin(phi)/cos(theta),cos(phi)/cos(theta)];
    
    % R3=Rb(3,:)
    R3=[cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);...
        cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);...
        cos(phi)*cos(theta)];
    
    nu_1=[u;v;w];
    nu_2=[p;q;r];
    kesi_1=[xc;yc;zc];
    dkesi_1=Rb*nu_1;
    kesi_2=[phi;theta;psi];
    dkesi_2=T*nu_2;
    
    dphi=dkesi_2(1);
    dtheta=dkesi_2(2);
    dpsi=dkesi_2(3);
    % Dynamics' matrices---------------------------------------------------
    g=[0;0;g0];
    
    J=[Ixx,0,-Ixx*sin(theta);...
        0,Iyy*cos(phi)^2-Izz*(cos(phi)^2-1),(sin(2*phi)*cos(theta)*(Iyy-Izz))/2;...
        -Ixx*sin(theta),(sin(2*phi)*cos(theta)*(Iyy-Izz))/2,Izz*cos(phi)^2*cos(theta)^2-Ixx*(cos(theta)^2-1)-Iyy*cos(theta)^2*(cos(phi)^2-1)];
    
    C=[0,((dtheta*sin(2*phi)-dpsi*cos(2*phi)*cos(theta))*(Iyy-Izz))/2,-dtheta*(Ixx*cos(theta)+(cos(theta)*(Iyy-Izz)*(2*cos(phi)^2-1))/2)-dpsi*cos(phi)*cos(theta)^2*sin(phi)*(Iyy-Izz);...
        (Ixx*dpsi*cos(theta))/2,-(sin(2*phi)*(Iyy-Izz)*(4*dphi-dpsi*sin(theta)))/4,dphi*((Ixx*cos(theta))/2+cos(2*phi)*cos(theta)*(Iyy-Izz))-(dtheta*sin(2*phi)*sin(theta)*(Iyy-Izz))/4+dpsi*cos(theta)*sin(theta)*(Iyy/2-Ixx+Izz/2-(Iyy*cos(2*phi))/2+(Izz*cos(2*phi))/2);...
        -Ixx*dtheta*cos(theta),((2*dphi*cos(2*phi)*cos(theta)-dtheta*sin(2*phi)*sin(theta))*(Iyy-Izz))/2,(dphi*sin(2*phi)*(2*Iyy-2*Izz)*(cos(2*theta)/2+1/2))/2-dtheta*sin(2*theta)*(Iyy/2-Ixx+Izz/2-(Iyy*cos(2*phi))/2+(Izz*cos(2*phi))/2)];
    % Control--------------------------------------------------------------
    % Translational control---------------------
    U=-KpT*(kesi_1-kesi_1_des)-KdT*(dkesi_1-dkesi_1_des);
    Tb(i)=m*(R3(1)*U(1)+R3(2)*U(2)+R3(3)*(U(3)+g0));
    % Orientation control-----------------------
    theta_des=atan((U(1)*cos(psi_des)+U(2)*sin(psi_des))/(U(3)+g0));
    phi_des=asin((U(1)*sin(psi_des)-U(2)*cos(psi_des))/(sqrt(U(1)^2+U(2)^2+(U(3)+g0)^2)));
    kesi_2_des=[phi_des;theta_des;psi_des];
    tau(:,i)=-KpO*(kesi_2-kesi_2_des)-KdO*(dkesi_2-dkesi_2_des);
    % System---------------------------------------------------------------
    x(:,i+1)=x(:,i)+[Rb*nu_1;...
        T*nu_2;...
        1/m*eye(3)*(R3*Tb(i)-D*dkesi_1-m*g);...
        inv(J)*(tau(:,i)-C*dkesi_2)]*Nt;
end
% figure(1)
% hold all
% plot(t,x(1,1:length(t)),'Linewidth',1.5)
% line([0,tf],[kesi_1_des(1),kesi_1_des(1)],'Color','magenta','LineStyle',':','Linewidth',1.5)
% xlabel('t (s)')
% ylabel('x_c(t) (m)')
% grid on
% box on
% figure(2)
% hold all
% plot(t,x(2,1:length(t)),'Linewidth',1.5)
% line([0,tf],[kesi_1_des(2),kesi_1_des(2)],'Color','magenta','LineStyle',':','Linewidth',1.5)
% xlabel('t (s)')
% ylabel('y_c(t) (m)')
% grid on
% box on
% figure(3)
% hold all
% plot(t,x(3,1:length(t)),'Linewidth',1.5)
% line([0,tf],[kesi_1_des(3),kesi_1_des(3)],'Color','magenta','LineStyle',':','Linewidth',1.5)
% xlabel('t (s)')
% ylabel('z_c(t) (m)')
% grid on
% box on
% figure(4)
% hold all
% plot(t,x(4,1:length(t)),'Linewidth',1.5)
% xlabel('t (s)')
% ylabel('\phi(t) (rad)')
% grid on
% box on
% figure(5)
% hold all
% plot(t,x(5,1:length(t)),'Linewidth',1.5)
% xlabel('t (s)')
% ylabel('\theta(t) (rad)')
% grid on
% box on
% figure(6)
% hold all
% plot(t,x(6,1:length(t)),'Linewidth',1.5)
% xlabel('t (s)')
% ylabel('\psi(t) (rad)')
% grid on
% box on
% figure(7)
% hold all
% plot(t,x(7,1:length(t)),'Linewidth',1.5)
% xlabel('t (s)')
% ylabel('u(t) (m/s)')
% grid on
% box on
% figure(8)
% hold all
% plot(t,x(8,1:length(t)),'Linewidth',1.5)
% xlabel('t (s)')
% ylabel('v(t) (m/s)')
% grid on
% box on
% figure(9)
% hold all
% plot(t,x(9,1:length(t)),'Linewidth',1.5)
% xlabel('t (s)')
% ylabel('w(t) (m/s)')
% grid on
% box on
% figure(10)
% hold all
% plot(t,x(10,1:length(t)),'Linewidth',1.5)
% xlabel('t (s)')
% ylabel('p(t) (rad/s)')
% grid on
% box on
% figure(11)
% hold all
% plot(t,x(11,1:length(t)),'Linewidth',1.5)
% xlabel('t (s)')
% ylabel('q(t) (rad/s)')
% grid on
% box on
% figure(12)
% hold all
% plot(t,x(12,1:length(t)),'Linewidth',1.5)
% xlabel('t (s)')
% ylabel('r(t) (rad/s)')
% grid on
% box on
% xc=x(1,1:length(t));
% yc=x(2,1:length(t));
% zc=x(3,1:length(t));
% phi=x(4,1:length(t));
% theta=x(5,1:length(t));
% psi=x(6,1:length(t));
% figure(13)
% hold all
% plot3(xc_0,yc_0,zc_0,'*r')
% plot3(kesi_1_des(1),kesi_1_des(2),kesi_1_des(3),'or')
% plot3(x(1,1:length(t)),x(2,1:length(t)),x(3,1:length(t)),'Linewidth',1.5)
% for i=1:100:N
%     xm1=xc(i)+L*cos(psi(i))*cos(theta(i));
%     ym1=yc(i)+L*cos(theta(i))*sin(psi(i));
%     zm1=zc(i)-L*sin(theta(i));
%     xm2=xc(i)+L*(cos(phi(i))*sin(psi(i))-cos(psi(i))*sin(phi(i))*sin(theta(i)));
%     ym2=yc(i)-L*(cos(phi(i))*cos(psi(i))+sin(phi(i))*sin(psi(i))*sin(theta(i)));
%     zm2=zc(i)-L*cos(theta(i))*sin(phi(i));
%     xm3=xc(i)-L*cos(psi(i))*cos(theta(i));
%     ym3=yc(i)-L*cos(theta(i))*sin(psi(i));
%     zm3=zc(i)+L*sin(theta(i));
%     xm4=xc(i)-L*(cos(phi(i))*sin(psi(i))-cos(psi(i))*sin(phi(i))*sin(theta(i)));
%     ym4=yc(i)+L*(cos(phi(i))*cos(psi(i))+sin(phi(i))*sin(psi(i))*sin(theta(i)));
%     zm4=zc(i)+L*cos(theta(i))*sin(phi(i));
%     line([xc(i);xm1],[yc(i);ym1],[zc(i);zm1],'color',[0.501960813999176 0.501960813999176 0.501960813999176])
%     line([xc(i);xm2],[yc(i);ym2],[zc(i);zm2],'color',[0.501960813999176 0.501960813999176 0.501960813999176])
%     line([xc(i);xm3],[yc(i);ym3],[zc(i);zm3],'color',[0.501960813999176 0.501960813999176 0.501960813999176])
%     line([xc(i);xm4],[yc(i);ym4],[zc(i);zm4],'color',[0.501960813999176 0.501960813999176 0.501960813999176])
%     %-------------------------Plot the wings-------------------------------
%     [xx,yy,zz]=cylinder;
%     w1=surf(R*xx+xm1,R*yy+ym1,-thickness*zz+zm1);
%     rotate(w1,[1,0,0],phi(i)*180/pi,[xm1,ym1,zm1]);
%     rotate(w1,[0,1,0],theta(i)*180/pi,[xm1,ym1,zm1]);
%     rotate(w1,[0,0,1],psi(i)*180/pi,[xm1,ym1,zm1]);
%     set(w1,'FaceColor',[1,0,0],'EdgeColor',[1,0,0]);
% 
%     w2=surf(R*xx+xm2,R*yy+ym2,-thickness*zz+zm2);
%     rotate(w2,[1,0,0],phi(i)*180/pi,[xm2,ym2,zm2]);
%     rotate(w2,[0,1,0],theta(i)*180/pi,[xm2,ym2,zm2]);
%     rotate(w2,[0,0,1],psi(i)*180/pi,[xm2,ym2,zm2]);
%     set(w2,'FaceColor',[0,0.5,1],'EdgeColor',[0,0.5,1]);
% 
%     w3=surf(R*xx+xm3,R*yy+ym3,-thickness*zz+zm3);
%     rotate(w3,[1,0,0],phi(i)*180/pi,[xm3,ym3,zm3]);
%     rotate(w3,[0,1,0],theta(i)*180/pi,[xm3,ym3,zm3]);
%     rotate(w3,[0,0,1],psi(i)*180/pi,[xm3,ym3,zm3]);
%     set(w3,'FaceColor',[1,0.5,0.5],'EdgeColor',[1,0.5,0.5]);
% 
%     w4=surf(R*xx+xm4,R*yy+ym4,-thickness*zz+zm4);
%     rotate(w4,[1,0,0],phi(i)*180/pi,[xm4,ym4,zm4]);
%     rotate(w4,[0,1,0],theta(i)*180/pi,[xm4,ym4,zm4]);
%     rotate(w4,[0,0,1],psi(i)*180/pi,[xm4,ym4,zm4]);
%     set(w4,'FaceColor',[0,0,1],'EdgeColor',[0,0,1]);
% end
% xlabel('X(m)')
% ylabel('Y(m)')
% zlabel('Z(m)')
% axis equal
% grid on
% box on
% view([1,1,1])
% figure(14)
% hold all
% plot(t,Tb,'Linewidth',1.5)
% xlabel('t (s)')
% ylabel('T_b(t) (N)')
% grid on
% box on
% figure(15)
% hold all
% plot(t,tau(1,:),'Linewidth',1.5)
% xlabel('t (s)')
% ylabel('\tau_\phi(t) (N.m)')
% grid on
% box on
% figure(16)
% hold all
% plot(t,tau(2,:),'Linewidth',1.5)
% xlabel('t (s)')
% ylabel('\tau_\theta(t) (N.m)')
% grid on
% box on
% figure(17)
% hold all
% plot(t,tau(3,:),'Linewidth',1.5)
% xlabel('t (s)')
% ylabel('\tau_\psi(t) (N.m)')
% grid on
% box on