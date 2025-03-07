clc;
clear;
data = load('pos_record.txt');%z_record_pos7  meas_pred_res_time_4   tool_force_data
master_x = data(:,1);
master_y = data(:,2);
master_z = data(:,3);
slave_x = data(:,4);
slave_y = data(:,5);
slave_z = data(:,6);
[n,m]=size(master_x);
distance = zeros(n,1);
for k =1 :n
    distance(k) = sqrt((master_x(k)-slave_x(k))^2+(master_y(k)-slave_y(k))^2+(master_z(k)-slave_z(k))^2);
end
    figure(1)
plot(distance,'b')
ylabel('米') 