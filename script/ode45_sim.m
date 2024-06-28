clc
clear all;
close all;
%%%%%%%%%%%%%%%
% Num = 5000;
% stepn = 10;
% x = 0:stepn:Num;
% y = 5000*sin(0.005*x);
% way_points = [x' y'];
%��ѡ����������·�߷ֽ��·����

way_points = [[100,100];[500,300];[900,500];[1300,900];[900,900];[500,1100];[100,1300];[500,2000]];%�ֶ�������·����
Npoints = way_points(:,1);%·���㱱����
Epoints = way_points(:,2);%·���㶫����
total = size(way_points); %·������������������·��ת��ʹ��
total = total(1);

%%%%%%%%%%%%%%%%
h=.5;%���� 
m=10000; %�ܹ����е���  p.s. ����ʱ�� = ���е���*����
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%
For = 0; %����ʼ��
Tor = 0;%���س�ʼ��
s = rng(23);
Cur = 1*randn(1);%��׼��Ϊ1�ĺ����ٶ�
Dir = 360/180*pi*rand(1);%��׼��Ϊ360��ĺ������򣬼�����������
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%
K_p = 50; K_i = 2.1; K_d = 0; %���ƿ����� PID�������ɵ���pid����ʹ����Ч����
%��
K_p1 = 100; K_i1 = 1; K_d1 = 0; %������PID�������ɵ���pid����ʹ����Ч������
beta = 3/180*pi;
velocity = 1;
FOR_MAX = 30;
TOR_MAX = 150;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ud = velocity; psid = 90*pi/180; %�����ٶȣ���������
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ue = 0; ue1 = 0; ue2 = 0; %��ʼ���ٶ����
psie = 0; psie1 = 0; psie2 = 0; %��ʼ���������
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Lnpp = 50;
R0=adative_radius(way_points,100,50,3,total);
V0=adative_velocity(way_points,2,1,2,total);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%������ʼ��
Force = zeros(m,1);
Torque = zeros(m,1);
Error = zeros(m,1);
UD = zeros(m,1);
PSID = zeros(m,1);
BETA = zeros(m,1);
UERROR = zeros(m,1);
Output = zeros(m,1);
x = zeros(6,1); %��ʱ״̬����
T=zeros(m,1); %ʱ��
Y=zeros(m,6); %״̬����
k = 1;
%%%%AUV״̬��ʼ��
u0 = 0;
v0 = 0;
r0 = 0;
x0 = 0;
y0 = 0;
psi0 = 80*pi/180;
Initial = [u0;v0;r0;x0;y0;psi0];
x = Initial;
%%%%%%%��ʼ����ѭ��
for i=1:1:m
t = h*i;
T(i,1)=t;
%%%%%%ʹ���Ľ�������������������˶�ѧ�붯��ѧ΢�ַ��̣�u v r x y psi
%�ĵ��������AUV��״̬��u v r x y psi ������Ҳ��ֱ��ʹ��ode45����������⣩
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
Y(i,6) = x(6,1);%��ת��
%%%������ǽ������ƣ�ʹ����0-2*pi֮��
while Y(i,6) > 2*pi
Y(i,6) = Y(i,6)- 2*pi;
end
while Y(i,6) < 0
Y(i,6) = Y(i,6) + 2*pi;
end
%%%%ʹ��������PID�㷨����ʵ��״̬������״̬�Ĳ�ֵ��Ϊ���������������
%����Ϊ���
ue2 = ue1;%���� PID�����������������
ue1 = ue;
ud = velocity;
ue = ud-Y(i,1);
For = For+K_p*(ue-ue1)+K_i*ue+K_d*(ue-2*ue1+ue2); %�����������Ƶ� PID ����
%���������ٶ���������������Ĵ�С
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
delta_u = K_p1*(psie-psie1)+K_d1*(psie-2*psie1+psie2);%������ PID �������������������������������
else
delta_u = K_p1*(psie-psie1)+K_i1*psie+K_d1*(psie-2*psie1+psie2);%������ PID ����
end

Tor = Tor + delta_u;


if Tor > TOR_MAX
Tor = TOR_MAX;
elseif Tor < -TOR_MAX
Tor = -TOR_MAX;
end


[isend,k,velocity] = point(way_points,Y(i,4),Y(i,5),k,total,R0,V0);
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
UD(i,1) = ud;
PSID(i,1) = psid/pi*180;
BETA(i,1) = beta;
UERROR(i,1) = ue;
end
rad2deg = 180/pi;
U =Y(:,1);
V =Y(:,2);
R =Y(:,3);
Xp =Y(:,4);
Yp =Y(:,5);
Psi = Y(:,6);

%%%%��ͼ
figure(1); %%%�����������������
subplot(2,1,1);plot(T,Force,'b','LineWidth',2);xlabel('t [s]');ylabel('F[N]');grid on;
subplot(2,1,2);plot(T,Torque,'b','LineWidth',2);xlabel('t [s]');ylabel('G [N.m]');grid on;
figure(2);%%%%%%%%%%% u��v��r
subplot(3,1,1);plot(T,U,'b','LineWidth',2);ylim([0 5]);xlabel('t[s]');ylabel('u(forw.vel.)[m/s]');
grid on;
subplot(3,1,2);plot(T,V,'b','LineWidth',2);xlabel('t [s]');ylabel('v(side slip vel.)[m/s]');grid on;
subplot(3,1,3);plot(T,R,'b','LineWidth',2);xlabel('t [s]');ylabel('r(yaw vel.)[rad/s2]');grid on;
title('Robot velocities in surge, sway and yaw');
figure(3);%%% x,y ����ʵ�ʹ켣�������켣
subplot(211);plot(Xp,Yp,'b','LineWidth',2);xlabel(' �� �� λ �� [m]');ylabel(' �� �� λ ��[m]');title('UUV ����');grid on;hold on;plot(Npoints,Epoints,'r--');legend('ʵ�ʺ���','��������');grid on;
subplot(212);plot(T,BETA,'b','LineWidth',2);grid on;xlabel('����ʱ��');ylabel('����������(m)');
figure(4);
subplot(211);plot(T,PSID,'b','LineWidth',2);grid on;xlabel('����ʱ��');ylabel('��������(��)');
subplot(212);plot(T,Error,'b','LineWidth',2);grid on;xlabel('����ʱ��');ylabel('�������(��)');
figure(5);
subplot(211);plot(T,UD,'b','LineWidth',2);grid on;xlabel('����ʱ��');ylabel('�����ٶ�(m/s)');
subplot(212);plot(T,UERROR,'b','LineWidth',2);grid on;xlabel('����ʱ��');ylabel('�ٶ����(m/s)');