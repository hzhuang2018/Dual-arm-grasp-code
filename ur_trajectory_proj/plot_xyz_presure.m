clc;
clear;
data = load('R_force.txt');%z_record_pos7  meas_pred_res_time_4   tool_force_data
%time = data(:,7);
f_x_tool = data(:,2);
f_y_tool = data(:,4);
f_z_tool = data(:,6);

f_x_base = data(:,1);
f_y_base = data(:,3);
f_z_base = data(:,5);

ft_x_base = data(:,7);
ft_y_base = data(:,9);
ft_z_base = data(:,11);
%d_time = diff(time);

figure(1)
plot(f_x_tool,'r.');
grid on;
%xlabel('时间/s') 
hold on;
plot(f_y_tool,'g.');
hold on;
plot(f_z_tool,'b.');
hold off;
ylabel('力/N'); 

figure(2)
plot(f_x_base,'r.');
grid on;
%xlabel('时间/s') 
hold on;
plot(f_y_base,'g.');
hold on;
plot(f_z_base,'b.');
hold off;
ylabel('力/N'); 


figure(3)
plot(ft_x_base,'r.');
grid on;
xlabel('时间/s') 
hold on;
plot(ft_y_base,'g.');
hold on;
plot(ft_z_base,'b.');
hold off;
ylabel('力/N'); 

% figure(2)
% plot(d_time*1e3,'b.')
% ylabel('时间间隔/ms') 


% for(inc =3:3)
%    figure;
%    grid on;
%    % 标题标注 
%     title('正常数据消耗时间') 
%     % 坐标轴标注 
%     xlabel('数据个数') 
%     ylabel('时间毫秒') 
%    hold on;
%    sigle_pixel = data(:,inc);
%    plot(sigle_pixel,'r');
%    %legend('压力数据');
%    hold off;
% end