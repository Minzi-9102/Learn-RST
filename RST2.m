% 创建一个刚体树对象，指定数据格式为列向量，最大刚体数为3
robot = rigidBodyTree('DataFormat','column','MaxNumBodies',3);
% 定义连杆长度
L1 = 0.3;% 第一段连杆长度
L2 = 0.3;% 第二段连杆长度
% 创建第一个连杆(link1)和关节(joint1)
body = rigidBody('link1');% 创建名为'link1'的刚体
joint = rigidBodyJoint('joint1', 'revolute');% 创建旋转关节
setFixedTransform(joint,trvec2tform([0 0 0]));% 设置关节的固定变换(无偏移)
joint.JointAxis = [0 0 1];% 设置关节旋转轴为Z轴
body.Joint = joint;% 将关节分配给刚体
addBody(robot, body, 'base');% 将刚体添加到机器人，连接到基座
% 创建第二个连杆(link2)和关节(joint2)
body = rigidBody('link2');% 创建名为'link2'的刚体
joint = rigidBodyJoint('joint2','revolute');% 创建旋转关节
setFixedTransform(joint, trvec2tform([L1,0,0]));% 设置关节固定变换(X轴偏移L1)
joint.JointAxis = [0 0 1];% 设置关节旋转轴为Z轴
body.Joint = joint;% 将关节分配给刚体
addBody(robot, body, 'link1');% 将刚体添加到机器人，连接到link1
% 创建末端执行器(tool)和固定关节
body = rigidBody('tool');% 创建名为'tool'的刚体(末端执行器)
joint = rigidBodyJoint('fix1','fixed');% 创建固定关节
setFixedTransform(joint, trvec2tform([L2, 0, 0]));% 设置固定变换(X轴偏移L2)
body.Joint = joint;% 将关节分配给刚体
addBody(robot, body, 'link2');% 将刚体添加到机器人，连接到link2
% 显示机器人详细信息
showdetails(robot)

t = (0:1/60:10)'; % 创建时间向量(0到10秒，间隔1/60秒)
count = length(t);% 轨迹点数量
% 定义圆形轨迹参数
center = [0.45 0.0 0];% 圆心坐标
radius = 0.15;% 圆半径
theta = t*(4*pi/t(end));% 角度向量(0到2π)，4pi表示转两圈
% 计算轨迹点坐标(在XY平面上的圆)
points = center + radius*[cos(theta) sin(theta) zeros(size(theta))];

% 获取机器人的初始(零位)配置
q0 = homeConfiguration(robot);
ndof = length(q0);% 自由度数量
qs = zeros(count, ndof);% 初始化关节角度矩阵

% 创建逆运动学求解器
ik = inverseKinematics('RigidBodyTree', robot);
weights = [0, 0, 0, 1, 1, 0];% 权重向量(仅考虑X和Y位置)
endEffector = 'tool';% 指定末端执行器

qInitial = q0; % 使用零位配置作为初始猜测
for i = 1:count
    % 求解使末端执行器到达目标点的关节配置
    % position
    point = points(i,:);% 当前目标点
    qSol = ik(endEffector,trvec2tform(point),weights,qInitial);
    % 存储求解得到的关节角度
    qs(i,:) = qSol;
    % 使用当前解作为下一次求解的初始猜测
    qInitial = qSol;
end

% 创建图形窗口并显示初始配置
figure
show(robot,qs(1,:)');
view(2)% 二维视图
ax = gca;
ax.Projection = 'orthographic';% 正交投影
hold on
plot(points(:,1),points(:,2),'k')% 绘制目标轨迹
axis([-0.1 0.7 -0.3 0.5])% 设置坐标轴范围
% 设置动画帧率
framesPerSecond = 60;
r = rateControl(framesPerSecond);

% 播放动画
for i = 1:count
    show(robot,qs(i,:)','PreservePlot',false);% 显示当前配置
    drawnow% 刷新图形
    waitfor(r);% 控制帧率
end