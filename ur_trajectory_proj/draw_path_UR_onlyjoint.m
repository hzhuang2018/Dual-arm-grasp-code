clc;
clear;
%%机械臂末端轨迹%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

q = load('pathB.txt');%%机械臂六个关节角
[Numq,nq] = size(q);

%%画障碍物相关数据%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
originPoint1=[0.68,-0.11,0.];%%障碍物原点%%    中心[0.84,0.,0.215]
cuboidSize1=[0.32,0.22,0.377];%%障碍物长宽高
originPoint2=[0.6497,0.5677,0.];%%障碍物原点%%  中心[0.9897,0.6352,0.213]
cuboidSize2=[0.68,0.135,0.4257];%%障碍物长宽高
% %% 函数PlotObstacle功能： 绘制长方体
% % 输入：
% %       originPoint：长方体的原点,行向量，如[0，0，0];
% %       cuboidSize：长方体的长宽高,行向量，如[10，20，30];
% % 输出：长方体图形

%%机械臂建模%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%通用格式L = Link（[theta d a alpha offset],CONVENTION）
%建立机器人模型
%          theta	d        a       alpha      offset
L(1)=Link([0      0.1807     0         pi/2        0],'standard'); %定义连杆的D-H参数
L(2)=Link([0      0          -0.6127   0           0],'standard');
L(3)=Link([0      0          -0.57155  0           0],'standard');
L(4)=Link([0      0.17415    0         pi/2        0],'standard');
L(5)=Link([0      0.11985    0         -pi/2       0],'standard');
L(6)=Link([0      0.11655    0         0           0],'standard');
robot=SerialLink(L,'name','UR10e'); %连接连杆，机器人取名UR10e

figure(1)
PlotObstacle(originPoint1,cuboidSize1)
hold on
PlotObstacle(originPoint2,cuboidSize2)
hold on
robot.plot(q);
grid on
xlabel('X');
ylabel('Y');
zlabel('Z');
% axis([-0.2 1.5 -1.2 1.2 -0.1 1.2]);
hold off
% robot.plot(q,'trail','b-','movie','仿真轨迹图.gif');%保存所画机械臂的动态轨迹图
