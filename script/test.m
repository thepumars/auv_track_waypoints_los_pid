%% initial paramter
position=[0;5;pi/4];
condition=[0;0;0];
force=[0;0;0];
predict_velocity=2;
%pid paramter
k_p=diag([6000,0,1000]);
k_i=diag([2,0,0]);
k_d=diag([200,0,35000]);
err_current=[0;0;0];
err_integral=[0;0;0];
delta_t=0.5;
%% trace
%坐标点
x=[0,100,200,400];
y=[0,100,300,400];
len=length(x);
trace=[x',y'];
point_storage=[position(1),position(2)];
angle_storage=position(3);
expect_angle_storage=0;
%los paramter
delta=1;
%% loop
i=1;
for j=1:2400
    %los
    err_y=trace(i+1,1)-trace(i,1);
    err_x=trace(i+1,2)-trace(i,2);
    whole_angle=atan2(err_y,err_x);
    trans=[cos(whole_angle),-sin(whole_angle);sin(whole_angle),cos(whole_angle)];   
    appendage_coordinate=trans'*[position(2)-trace(i,2);position(1)-trace(i,1)];
    rang=abs(trans'*[position(2)-trace(i+1,2);position(1)-trace(i+1,1)]);
    predict_path_angle=whole_angle-atan(appendage_coordinate(2)/delta);
    predict_path_angle=pi/2-predict_path_angle;
    if rang(1)<delta
        i=i+1;
        err_integral=[0;0;0];
    end
    %% pid-->force
    err_angle=(predict_path_angle-position(3))/pi*180;
    err_velocity=predict_velocity-condition(1);
    err_last=err_current;
    err_current=[err_velocity;0;err_angle];
    err_integral=err_integral+err_current;
    force_before=[force(1:2,1);0];
    force=k_p*err_current+k_i*err_integral+k_d*(err_current-err_last);
    %condition & position
    acceleration=ship_model(position,condition,force);
    condition=condition+acceleration*delta_t;
    angle=position(3);
    J=[cos(angle),-sin(angle),0;
       sin(angle),cos(angle),0;
       0,0,1];
    position=position+J*condition*delta_t;
    if position(3)>pi 
        position(3)=position(3)-2*pi;
    end
    if position(3)<-pi 
        position(3)=2*pi+position(3);
    end
    point_storage=[point_storage;position(1),position(2)];
    angle_storage=[angle_storage;position(3)];
    expect_angle_storage=[expect_angle_storage;predict_path_angle];
end
%% draw
figure(1)
plot(trace(:,1),trace(:,2),'b.-');
hold on;
plot(point_storage(:,1),point_storage(:,2),'r');
figure(2)
len_angle=length(angle_storage);
angle_2=0:len_angle-1;
angle_storage=[angle_storage,angle_2'];
expect_angle_storage=[expect_angle_storage,angle_2'];
plot(angle_storage(:,2),angle_storage(:,1),'r');
hold on;
plot(expect_angle_storage(:,2),expect_angle_storage(:,1),'b');
%% simplified ship mathematical model
function acceleration=ship_model(position,condition,force)
%poistion=[x;y;angle]---condition=[u;v;r]--force=[F1;F2;M]
%ship basic paramter
ship_length=38;%m
ship_m=118000;%kg
m_11=120000;
m_22=177900;
m_33=63600000;%kg.m^2
d_11=21500;%kg/s
d_22=147000;%kg/s
d_33=8020000;%kg.m^2/s
angle=position(3);
u=condition(1);
v=condition(2);
%matrix
J=[cos(angle),-sin(angle),0;
   sin(angle),cos(angle),0;
   0,0,1];
M=[m_11,0,0;
   0,m_22,0;
   0,0,m_33];
C=[0,0,-m_22*v;
   0,0,m_11*u;
   m_22*v,-m_11*u,0];
D=[d_11,0,0;
   0,d_22,0;
   0,0,d_33];
acceleration=M\(-C*condition-D*condition+force);
end