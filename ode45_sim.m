clc
clear all;
close all;
%%%%%%%%%%%%%%%
way_points = [[100,100];[2100,100];[2100,300];[100,300];[100,500];[2100,500];[2100,700];[100,900]];
Npoints = way_points(:,1);
Epoints = way_points(:,2);
total = size(way_points);
total = total(1);
%%%%%%%%%%%%%%%%
h=.5;
%����
m=40000; %�ܹ�����ʱ��
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%
For = 0; %����ʼ��
Tor = 0;
%���س�ʼ��
s = rng(23);
Cur = 1*randn(1);
Dir = 180/180*pi*rand(1);
%������С���ɸ���ʵ�����������
%��������
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%
K_p = 50; K_i = 1.1; K_d = 0; %���ƿ����� PID�������ɵ���pid����ʹ����Ч����
%��
K_p1 = 100; K_i1 = 10; K_d1 = 10; %������PID�������ɵ���pid����ʹ����Ч������
beta = 5;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ud = 1; psid = 90*pi/180; %�����ٶȣ���������
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ue = 0; ue1 = 0; ue2 = 0; %��ʼ���ٶ����
psie = 0; psie1 = 0; psie2 = 0; %��ʼ���������
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Lnpp = 100;
R0 = 50;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%������ʼ��
Force = zeros(m,1);
Torque = zeros(m,1);
Error = zeros(m,1);
PSID = zeros(m,1);
Output = zeros(m,1);
x = zeros(6,1); %��ʱ״̬����
T=zeros(m,1); %ʱ��
Y=zeros(m,6); %״̬����
k = 1;
%%%%AUV״̬��ʼ��
u0 = 0;
v0 = 0;
r0 = 0;
x0 = 30;
y0 = 30;
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
ue = ud-Y(i,1);
For = For+K_p*(ue-ue1)+K_i*ue+K_d*(ue-2*ue1+ue2); %�����������Ƶ� PID ����
%���������ٶ���������������Ĵ�С
psie2 = psie1;
psie1 = psie;
psie = psid- Y(i,6);
if psie > pi
psie = psie- 2*pi;
elseif psie <-pi
psie = psie + 2*pi;
end

Error(i,1) = psie;
if psie > beta
Tor = Tor+K_p1*(psie-psie1)+K_d1*(psie-2*psie1+psie2);%������ PID �������������������������������
else
Tor = Tor+K_p1*(psie-psie1)+K_i1*psie+K_d1*(psie-2*psie1+psie2);%������ PID ����
end



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
psid = los_angle(way_points,Y(i,4),Y(i,5),k,8,Lnpp);
end

PSID(i,1) = psid/pi*180;

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
subplot(212);plot(T,PSID,'r--','LineWidth',2);grid on;xlabel('����ʱ��');ylabel('���(m)');