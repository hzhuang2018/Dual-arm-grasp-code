clc;
clear;
data = textread('tool_force_data22.txt');%z_record_pos7  meas_pred_res_time_4
for(inc =3:3)
   figure;
   grid on;
   % 标题标注 
    title('正常数据消耗时间') 
    % 坐标轴标注 
    xlabel('数据个数') 
    ylabel('时间毫秒') 
   hold on;
   sigle_pixel = data(:,inc);
   plot(sigle_pixel,'r');
   %legend('压力数据');
   hold off;
end