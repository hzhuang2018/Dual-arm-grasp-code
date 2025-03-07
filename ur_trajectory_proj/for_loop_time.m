clc;
clear;
data = load('for_loop_time.txt');%z_record_pos7  meas_pred_res_time_4   tool_force_data
time_pre = data(:,1);
%time_cur = data(:,2);
d_time= diff(time_pre);
%d_time = time_cur-time_pre;

figure(2)
plot(d_time*1e3,'b.')
ylabel('时间间隔/ms') 