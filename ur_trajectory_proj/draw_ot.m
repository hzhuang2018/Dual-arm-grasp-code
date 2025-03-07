% 读取 pcd 文件，并取出 xyz 坐标
ptCloud = load('simple_tree1.bt');
% 可视化显示当前 pcd 文件
pcshow(ptCloud);
% 将该文件的 xyz 坐标取出
point_xyz = ptCloud.Location;
% 分别取出 x, y, z 坐标，并转化为 double 型（一开始是 single 类型）
px = double(point_xyz(:,1));
py = double(point_xyz(:,2));
pz = double(point_xyz(:,3));
