clc;
clear;
data = load('path0.txt');%z_record_pos7  meas_pred_res_time_4   tool_force_data

x = data(:,1);
y = data(:,2);
z = data(:,3);

dataB = load('pathB.txt');%z_record_pos7  meas_pred_res_time_4   tool_force_data

xB = dataB(:,1);
yB = dataB(:,2);
zB = dataB(:,3);
%d_time = diff(time);

[xs,ys,zs]=sphere(40);
r=0.2;
xs=r*xs+0.84;%后面的0.4/0/0.2为球心坐标
ys=r*ys+0;
zs=r*zs+0.215;

figure(1)
plot3(x,y,z,'g')
hold on
plot3(xB,yB,zB,'r')
hold on
surf(xs,ys,zs)
grid on
axis([-1,1,-1,1,-1,1]);

