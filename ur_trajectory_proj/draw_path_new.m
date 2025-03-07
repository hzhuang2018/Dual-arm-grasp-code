clc;
clear;
%%机械臂末端轨迹%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
data = load('D:\WORK\robotic_grasp\robotic_grasp\ur_trajectory_proj\data_record\path0_05110953.txt');%z_record_pos7  meas_pred_res_time_4   tool_force_data
x = data(:,1);
y = data(:,2);
z = data(:,3);
dataB = load('D:\WORK\robotic_grasp\robotic_grasp\ur_trajectory_proj\data_record\pathB_05110953.txt');%z_record_pos7  meas_pred_res_time_4   tool_force_data
xB = dataB(:,1);
yB = dataB(:,2);
zB = dataB(:,3);
%d_time = diff(time);

%%画障碍物相关数据%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
originPoint=[0.68,-0.11,0.0225];%%障碍物原点%%中心[0.84,0.,0.215]
cuboidSize=[0.32,0.22,0.385];%%障碍物长宽高
%% 函数功能： 绘制长方体
% 输入：
%       originPoint：长方体的原点,行向量，如[0，0，0];
%       cuboidSize：长方体的长宽高,行向量，如[10，20，30];
% 输出：长方体图形
%% 根据原点和尺寸，计算长方体的8个的顶点
vertexIndex=[0 0 0;0 0 1;0 1 0;0 1 1;1 0 0;1 0 1;1 1 0;1 1 1];
vertex=originPoint+vertexIndex.*cuboidSize;
%% 定义6个平面分别对应的顶点
facet=[1 2 4 3;1 2 6 5;1 3 7 5;2 4 8 6;3 4 8 7;5 6 8 7];
%% 定义8个顶点的颜色，绘制的平面颜色根据顶点的颜色进行插补
color=[1;2;3;4;5;6;7;8];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure(1)
plot3(x,y,z,'g','LineWidth',2)
hold on
plot3(xB,yB,zB,'r','LineWidth',2)
hold on
patch('Vertices',vertex,'Faces',facet,'FaceVertexCData',color,'FaceColor','interp','FaceAlpha',0.5);
grid on
xlabel('X');
ylabel('Y');
zlabel('Z');
axis([0.2 1.2 -1.2 1.2 -0.1 1.2]);
