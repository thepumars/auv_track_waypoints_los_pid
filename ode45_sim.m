clc
clear all;
close all;
%%%%%%%%%%%%%%%
% Num = 5000;
% stepn = 100;
% x = 0:stepn:Num;
% y = 5000*sin(0.005*x);
% way_points = [x' y'];


way_points = [[100,100];[500,300];[900,500];[1300,900];[900,900];[500,1100];[100,1300];[500,2000]];
Npoints = way_points(:,1);
Epoints = way_points(:,2);
total = size(way_points); 
total = total(1);

%%%%%%%%%%%%%%%%
h=.5;
%步长
m=20000; %总共运行时间
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%
For = 0; %力初始化
Tor = 0;
%力矩初始化
s = rng(23);
Cur = 1*randn(1);
Dir = 180/180*pi*rand(1);
%海流大小（可根据实际情况调整）
%海流方向
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%
K_p = 50; K_i = 1.1; K_d = 0; %主推控制力 PID参数，可调整pid参数使控制效果更
%好
K_p1 = 100; K_i1 = 100; K_d1 = 10; %舵力矩PID参数，可调整pid参数使控制效果更好
beta = .5;

FOR_MAX = 1.5;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ud = 1; psid = 90*pi/180; %期望速度，期望艏向
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ue = 0; ue1 = 0; ue2 = 0; %初始化速度误差
psie = 0; psie1 = 0; psie2 = 0; %初始化艏向误差
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Lnpp = 50;
R0=adative_radius(way_points,50,20,1,total);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%变量初始化
Force = zeros(m,1);
Torque = zeros(m,1);
Error = zeros(m,1);
PSID = zeros(m,1);
BETA = zeros(m,1);
Output = zeros(m,1);
x = zeros(6,1); %临时状态变量
T=zeros(m,1); %时间
Y=zeros(m,6); %状态变量
k = 1;
%%%%AUV状态初始化
u0 = 0;
v0 = 0;
r0 = 0;
x0 = 0;
y0 = 0;
psi0 = 80*pi/180;
Initial = [u0;v0;r0;x0;y0;psi0];
x = Initial;
%%%%%%%开始进入循环
for i=1:1:m
t = h*i;
T(i,1)=t;
%%%%%%使用四阶龙格库塔法，根据其运动学与动力学微分方程（u v r x y psi
%的导数）求解AUV的状态（u v r x y psi ）。（也可直接使用ode45函数进行求解）
k1=Infante_horizontal(t,x,[For,Tor,Cur,Dir]);
k2=Infante_horizontal(t + h/2, x+h/2.*k1,[For,Tor,Cur,Dir]);
k3=Infante_horizontal(t + h/2, x+h/2.*k2,[For,Tor,Cur,Dir]);
k4=Infante_horizontal(t + h, x+h.*k3,[For,Tor,Cur,Dir]);
x =x+h/6.*(k1 + 2.*k2 + 2.*k3 + k4);
Force(i,1) = For;
Torque(i,1) = Tor;
Y(i,1) = x(1,1);
Y(i,2) = x(2,1);
Y(i,3) = x(3,1);
Y(i,4) = x(4,1);
Y(i,5) = x(5,1);
Y(i,6) = x(6,1);%回转角
%%%对艏向角进行限制，使其在0-2*pi之间
while Y(i,6) > 2*pi
Y(i,6) = Y(i,6)- 2*pi;
end
while Y(i,6) < 0
Y(i,6) = Y(i,6) + 2*pi;
end
%%%%使用增量型PID算法，将实际状态与期望状态的差值作为输出，控制力与力
%矩作为输出
ue2 = ue1;%这里 PID控制器的输入是误差
ue1 = ue;
ue = ud-Y(i,1);
For = For+K_p*(ue-ue1)+K_i*ue+K_d*(ue-2*ue1+ue2); %这里就是所设计的 PID 控制
%器，根据速度误差来调整推力的大小
if For > FOR_MAX
For = FOR_MAX;
elseif For < -FOR_MAX
For = -FOR_MAX;
end
psie2 = psie1;
psie1 = psie;
psie = psid- Y(i,6);
if psie > pi
psie = psie- 2*pi;
elseif psie <-pi
psie = psie + 2*pi;
end
% if abs(psie) < 5/180*pi
%     psie = 0;
% end
Error(i,1) = psie;
if psie > beta
delta_u = K_p1*(psie-psie1)+K_d1*(psie-2*psie1+psie2);%这里用 PID 控制器根据艏向角误差来调整舵的力矩
else
delta_u = K_p1*(psie-psie1)+K_i1*psie+K_d1*(psie-2*psie1+psie2);%这里用 PID 控制
end

Tor = Tor + delta_u;




[isend,k] = point(way_points,Y(i,4),Y(i,5),k,total,Lnpp,R0);
if isend == 1
    Y(i+1:end,1) = x(1,1);
    Y(i+1:end,2) = x(2,1);
    Y(i+1:end,3) = x(3,1);
    Y(i+1:end,4) = x(4,1);
    Y(i+1:end,5) = x(5,1);
    Y(i+1:end,6) = x(6,1);
    Force(i+1:end,1) = For;
    Torque(i+1:end,1) = Tor;
    
    T(i+1:end,1)=h*(i+1):h:h*m;
    break;
    
end
if mod(i,10) ==0
[beta,psid] = los_angle(way_points,Y(i,4),Y(i,5),k,total,Lnpp);
end

PSID(i,1) = psid/pi*180;
BETA(i,1) = beta;
end
rad2deg = 180/pi;
U =Y(:,1);
V =Y(:,2);
R =Y(:,3);
Xp =Y(:,4);
Yp =Y(:,5);
Psi = Y(:,6);

%%%%画图
figure(1); %%%控制器输出力与力矩
subplot(2,1,1);plot(T,Force,'b','LineWidth',2);xlabel('t [s]');ylabel('F[N]');grid on;
subplot(2,1,2);plot(T,Torque,'b','LineWidth',2);xlabel('t [s]');ylabel('G [N.m]');grid on;
figure(2);%%%%%%%%%%% u，v，r
subplot(3,1,1);plot(T,U,'b','LineWidth',2);ylim([0 5]);xlabel('t[s]');ylabel('u(forw.vel.)[m/s]');
grid on;
subplot(3,1,2);plot(T,V,'b','LineWidth',2);xlabel('t [s]');ylabel('v(side slip vel.)[m/s]');grid on;
subplot(3,1,3);plot(T,R,'b','LineWidth',2);xlabel('t [s]');ylabel('r(yaw vel.)[rad/s2]');grid on;
title('Robot velocities in surge, sway and yaw');
figure(3);%%% x,y 方向实际轨迹与期望轨迹
subplot(211);plot(Xp,Yp,'b','LineWidth',2);xlabel(' 东 向 位 移 [m]');ylabel(' 北 向 位 移[m]');title('UUV 航迹');grid on;hold on;plot(Npoints,Epoints,'r--');legend('实际航迹','期望航迹');grid on;
subplot(212);plot(T,BETA,'r--','LineWidth',2);grid on;xlabel('仿真时间');ylabel('误差(m)');
figure(4);
subplot(211);plot(T,PSID,'r--','LineWidth',2);grid on;xlabel('仿真时间');ylabel('期望航向(°)');
subplot(212);plot(T,Error,'r--','LineWidth',2);grid on;xlabel('仿真时间');ylabel('航向误差(°)');