function dydt=Infante_horizontal(t,x,formom)
%[u v r x y psi ]
%[T,Y] = Infante(t,x) returns the time derivative of the state vector:
%anAutnomous Underwater Vehicle (AUV) of Infante. While the state vector is defined as
%follows:
dydt = zeros(6,1);
ur = x(1);vr = x(2); r = x(3);xx=x(4);yy=x(5); psi = x(6); %x Ϊ��ǰʱ�� AUV��״̬[u v
%r x y psi ]
F =formom(1);%%PID�����
T=formom(2);%%?��PID�������
uc = formom(3)*( cos(formom(4))*cos(psi) + sin(formom(4))*sin(psi) ); %�����ٶȷ���
vc = formom(3)*(-cos(formom(4))*sin(psi) + sin(formom(4))*cos(psi) ); %�����ٶȷ���
u = ur+uc; %%AUV�ں���Ӱ���µ���ʵ�ٶ�
v = vr+vc; %%AUV�ں���Ӱ���µ���ʵ�ٶ�
%%%%%%AUVˮ����ϵ��
m=40;
Xdu =-5.42; Xu = 0.1; Xuu = 8.2;
Ydv =-38.4; Yv = 10; Yvv = 200; Ydr =-2.5; Yr = 5;
Ndr =-8.9; Ndv = 2.2; Nv = 36; Nr = 5; Nrr = 15;
Iz = 8.0;
Yuv = 0; Yur = u*Xdu; Nuv = u*(Ydv-Xdu); Nur = u*Ydr;
d11 = Xu+Xuu*abs(ur); d22 = Yv+Yvv*abs(vr); d66 = Nr+Nrr*abs(r); d26 = Yr; d62 = Nv;
c26 = m-Xdu; c62 = Xdu-Ydv;
m11=m-Xdu; m22 =m-Ydv; m26 =-Ydr; m66 =Iz-Ndr;
A=-d22*vr+(d26-ur*c26-m*uc)*r;
B=(d62-ur*c62)*vr-d66*r+T;
ur = x(1);vr = x(2); r = x(3);xx=x(4);yy=x(5); psi = x(6);
%%%%AUV����ѧģ��
dydt(1) = (1/m11)*(-d11*ur+F);
dydt(2) = (A*m66-B*m26)/(m22*m66-m26*m26);
dydt(3) = (B*m22-A*m26)/(m22*m66-m26*m26);
%%%AUV�˶�ѧģ��
dydt(4) = u*cos(psi)- v*sin(psi);
dydt(5) = u*sin(psi) + v*cos(psi);
dydt(6) = r ;