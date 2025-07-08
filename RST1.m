body1 = rigidBody('body1');%构建一个刚体
jnt1 = rigidBodyJoint('jnt1','revolute');%构建一个关节，revolute表示旋转关节
jnt1.HomePosition = pi/4;%设定初始角度为pi/4，即该刚体在初始时关节坐标系相对前一个坐标系的角度为pi/4
tform = trvec2tform([0.25, 0.25, 0]); %用齐次变换矩阵定义该刚体关节坐标系与前一关节坐标系的位姿关系
setFixedTransform(jnt1,tform);% 设置关节的固定变换
body1.Joint = jnt1;% 将关节分配给刚体
robot = rigidBodyTree;%创建刚体树
addBody(robot,body1,'base')%将body1连接到基座base，base的坐标系原点为（0，0，0）
%% 创建第二个刚体
body2 = rigidBody('body2');
jnt2 = rigidBodyJoint('jnt2','revolute');
jnt2.HomePosition = pi/6; % User defined
tform2 = trvec2tform([1, 0, 0]); % User defined
setFixedTransform(jnt2,tform2);
body2.Joint = jnt2;
addBody(robot,body2,'body1'); % Add body2 to body1
%% 创建第三、第四个刚体
body3 = rigidBody('body3');
body4 = rigidBody('body4');
jnt3 = rigidBodyJoint('jnt3','revolute');
jnt4 = rigidBodyJoint('jnt4','revolute');
tform3 = trvec2tform([0.6, -0.1, 0])*eul2tform([-pi/2, 0, 0]); % 先按欧拉角方式绕z轴旋转-pi/2，随后平移
tform4 = trvec2tform([1, 0, 0]); % User defined
setFixedTransform(jnt3,tform3);
setFixedTransform(jnt4,tform4);
jnt3.HomePosition = pi/4; % 在固定变换后再按照初始角度旋转
body3.Joint = jnt3
body4.Joint = jnt4
addBody(robot,body3,'body2'); % Add body3 to body2
addBody(robot,body4,'body2'); % Add body4 to body2
%% 添加末端执行器
bodyEndEffector = rigidBody('endeffector');
tform5 = trvec2tform([0.5, 0, 0]); % User defined
setFixedTransform(bodyEndEffector.Joint,tform5);
addBody(robot,bodyEndEffector,'body4');
%% 
config = randomConfiguration(robot)%生成一个随机的关节配置
tform = getTransform(robot,config,'endeffector','base')%计算正运动学，用齐次变换矩阵表示结果
%% 提取子树，并进行修改
newArm = subtree(robot,'body2');%从body2开始提取后续子树
removeBody(newArm,'body3');%删除body3
removeBody(newArm,'endeffector')%删除末端执行器
%% 
newBody1 = copy(getBody(newArm,'body2'));
newBody2 = copy(getBody(newArm,'body4'));
newBody1.Name = 'newBody1';
newBody2.Name = 'newBody2';
newBody1.Joint = rigidBodyJoint('newJnt1','revolute');
newBody2.Joint = rigidBodyJoint('newJnt2','revolute');
tformTree = trvec2tform([0.2, 0, 0]); % User defined
setFixedTransform(newBody1.Joint,tformTree);
replaceBody(newArm,'body2',newBody1);
replaceBody(newArm,'body4',newBody2);
%%
% 将子树添加到刚体上
addSubtree(robot,'body1',newArm);
showdetails(robot)