clc; clear;

% 加载ABB IRB 120机器人模型，设置数据格式为行向量，重力加速度为[0 0 -9.81]
robot = loadrobot("abbIrb120",'DataFormat','row','Gravity',[0 0 -9.81]);

% 获取机器人的初始关节配置（home位置）
% currentRobotJConfig = homeConfiguration(robot);

% 获取机器人关节数量
numJoints = numel(homeConfiguration(robot));

% 设置末端执行器名称
endEffector = "tool0";
%% 
% 设置时间步长（秒）
timeStep = 1/10; 

% 设置末端执行器移动速度（m/s）
toolSpeed = 0.1; 

% 创建逆运动学求解器
ik = inverseKinematics('RigidBodyTree',robot);
ik.SolverParameters.AllowRandomRestart = false; % 关闭随机重启
weights = [1 1 1 1 1 1]; % 设置逆运动学权重

% 获取初始末端执行器的位姿变换矩阵
% taskInit = getTransform(robot,jointInit,endEffector);
taskInit = trvec2tform([0.3,0,0.7])*axang2tform([0 1 0 0]);

% 初始关节位置
initialGuess = homeConfiguration(robot);
jointInit = ik(endEffector,taskInit,weights,initialGuess);
% show(robot,jointInit,'PreservePlot',false,'Frames','off');

% 定义目标位姿：位置偏移[0.2,0,0.2]
taskFinal = trvec2tform([0.3,0,0.2])*axang2tform([0 1 0 0]);

% 计算初始位姿和目标位姿之间的欧式距离
distance = norm(tform2trvec(taskInit)-tform2trvec(taskFinal));

% 设置运动时间参数
initTime = 0;
finalTime = (distance/toolSpeed) - initTime;

% 生成时间序列
trajTimes = initTime:timeStep:finalTime;
timeInterval = [trajTimes(1); trajTimes(end)];
%% 

% 生成任务空间的轨迹点和速度
[taskWaypoints,taskVelocities] = transformtraj(taskInit,taskFinal,timeInterval,trajTimes); 

% 预计算所有路径点对应的关节配置
jointConfigs = zeros(length(trajTimes), numJoints);
for i = 1:length(trajTimes)
    jointConfigs(i,:) = ik(endEffector, taskWaypoints(:,:,i), weights, initialGuess);
    initialGuess = jointConfigs(i,:); % 使用上一次的解作为初始猜测
end


% % 创建任务空间运动模型
% tsMotionModel = taskSpaceMotionModel('RigidBodyTree',robot,'EndEffectorName','tool0');
% % ⬆初始化一个任务空间控制器，为后续的笛卡尔空间轨迹规划和控制提供计算基础。
% % 它抽象了机器人的动力学特性，使得用户可以直接操作末端执行器的位姿，而无需手动处理关节级别的细节。
% 
% % 设置位置和速度控制增益（这里将位置和速度增益设为0，可能使用默认值）
% tsMotionModel.Kp(1:3,1:3) = 0;
% tsMotionModel.Kd(1:3,1:3) = 0;
% 
% % 设置初始关节位置和速度
% q0 = jointInit; 
% qd0 = zeros(size(q0));
% 
% % 使用ode15s求解器求解任务空间运动微分方程
% [tTask,stateTask] = ode15s(@(t,state) exampleHelperTimeBasedTaskInputs(tsMotionModel,timeInterval,taskInit,taskFinal,t,state),timeInterval,[q0; qd0]);
% %tTask：ode15s求解器返回的时间向量（N×1），记录了每个仿真时间步的时间点。
% %stateTask：ode15s求解器返回的状态矩阵（N×2M），其中：
% % 奇数列：各关节的位置（q）。
% % 偶数列：各关节的速度（qd）。
% % M 是机器人的关节数（numJoints）。

%% 

% % 创建逆运动学求解器
% ik = inverseKinematics('RigidBodyTree',robot);
% ik.SolverParameters.AllowRandomRestart = false; % 关闭随机重启
% weights = [1 1 1 1 1 1]; % 设置逆运动学权重
% 
% % 计算目标位姿对应的关节角度
% initialGuess = jointInit;
% jointFinal = ik(endEffector,taskFinal,weights,initialGuess);
% 
% % 将关节角度限制在[-pi,pi]范围内
% wrappedJointFinal = wrapToPi(jointFinal);

% % 生成关节空间的三次多项式轨迹
% ctrlpoints = [jointInit',wrappedJointFinal'];
% jointConfigArray = cubicpolytraj(ctrlpoints,timeInterval,trajTimes);

% % 创建关节空间运动模型（PD控制）
% jsMotionModel = jointSpaceMotionModel('RigidBodyTree',robot,'MotionType','PDControl');
% 
% % 设置初始关节位置和速度
% q0 = currentRobotJConfig; 
% qd0 = zeros(size(q0));
% 
% % 使用ode15s求解器求解关节空间运动微分方程
% [tJoint,stateJoint] = ode15s(@(t,state) exampleHelperTimeBasedJointInputs(jsMotionModel,timeInterval,jointConfigArray,t,state),timeInterval,[q0; qd0]);
%% 

% 创建图形窗口并设置为全屏
% f = figure('Name','机械臂轨迹规划演示','NumberTitle','off',...
%        'Units','normalized','Position',[0 0 1 1],...
%        'Color','w','MenuBar','none','ToolBar','none');

f = figure('Name','机械臂轨迹规划演示','NumberTitle','off',...
       'Units','normalized','Position',[0 0 1 1],...
       'Color','w','MenuBar','none','ToolBar','none');

% 创建位姿数据显示区域
posePanel = uipanel('Title','末端执行器位姿数据','FontSize',12,...
                   'Position',[0.7 0.7 0.25 0.25],...
                   'BackgroundColor','w');
               
% 创建位置数据显示文本框
posText = uicontrol('Parent',posePanel,'Style','text',...
                   'Units','normalized','Position',[0.05 0.6 0.9 0.3],...
                   'FontSize',12,'HorizontalAlignment','left',...
                   'String','位置 (x,y,z): [0.000, 0.000, 0.000] m',...
                   'BackgroundColor','w');

% 创建姿态数据显示文本框（欧拉角）
eulerText = uicontrol('Parent',posePanel,'Style','text',...
                     'Units','normalized','Position',[0.05 0.2 0.9 0.3],...
                     'FontSize',12,'HorizontalAlignment','left',...
                     'String','姿态 (ZYX欧拉角): [0.000, 0.000, 0.000] rad',...
                     'BackgroundColor','w');

% 设置循环播放次数（0表示无限循环）
loopCount = 0;
currentLoop = 0;

% 添加停止按钮
uicontrol('Style', 'pushbutton', 'String', '停止循环',...
    'Units','normalized','Position', [0.85 0.02 0.1 0.05],...
    'Callback', 'setappdata(gcf,''stopLoop'',true)');

% 设置停止标志
setappdata(f, 'stopLoop', false);
%% 

% 开始循环播放
while true
    if loopCount > 0 && currentLoop >= loopCount
        break;
    end
    
    % 检查是否点击了停止按钮
    if getappdata(f, 'stopLoop')
        break;
    end
    
    currentLoop = currentLoop + 1;

% 显示机器人初始配置
show(robot,jointInit,'PreservePlot',false,'Frames','off');
hold on
axis([-1 1 -1 1 -0.1 1.5]); % 设置坐标轴范围

grid on; % 添加网格
% view(3); % 设置3D视图
xlabel('X轴'); ylabel('Y轴'); zlabel('Z轴'); % 添加坐标轴标签
title(['机械臂轨迹规划演示 - 任务空间规划 (循环次数: ' num2str(currentLoop) ']'],'FontSize',14);
% 清除之前的轨迹点
delete(findobj(gca, 'Type', 'line', 'Marker', '.'));

% 动画演示任务空间轨迹
for i=1:length(trajTimes)

    if getappdata(f, 'stopLoop')
       break;
    end

    % % 当前时间 
    % tNow = trajTimes(i);
    % 获取当前时间的关节配置
    configNow = jointConfigs(i,:);
    % 获取当前末端执行器位姿
    poseNow = getTransform(robot,configNow,endEffector);

    % 更新位姿数据显示
    pos = tform2trvec(poseNow); % 获取位置向量
    rotm = tform2rotm(poseNow); % 获取旋转矩阵
    eul = rotm2eul(rotm, 'ZYX'); % 转换为ZYX欧拉角

    set(posText, 'String', sprintf('位置 (x,y,z): [%.3f, %.3f, %.3f] m', pos(1), pos(2), pos(3)));
    set(eulerText, 'String', sprintf('姿态 (ZYX欧拉角): [%.3f, %.3f, %.3f] rad', eul(1), eul(2), eul(3)));

    % 显示机器人当前配置
    show(robot,configNow,'PreservePlot',false,'Frames','off');
    % 绘制末端执行器位置轨迹（蓝色点）
    taskSpaceMarker = plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'b.','MarkerSize',20);
    drawnow;
end

    if getappdata(f, 'stopLoop')
        break;
    end
if ~getappdata(f, 'stopLoop')
        % legend([taskSpaceMarker jointSpaceMarker], {'任务空间规划', '关节空间规划'},...
        %        'Location','northeast','FontSize',12);
        title(['机械臂轨迹规划演示  (循环次数: ' num2str(currentLoop) ']'],'FontSize',14);
end
end
%% 

% 任务空间输入辅助函数
function stateDot = exampleHelperTimeBasedTaskInputs(motionModel,timeInterval,initialTform,finalTform,t,state)
    % 计算当前时间的参考位姿和速度
    [refPose,refVel] = transformtraj(initialTform,finalTform,timeInterval,t);
    % 计算机器人运动微分
    stateDot = derivative(motionModel,state,refPose,refVel);
end

% % 关节空间输入辅助函数
% function stateDot = exampleHelperTimeBasedJointInputs(motionModel, timeInterval,configWaypoints,t,state)
%     % 计算当前时间的关节位置和速度
%     [qd,qdDot] = bsplinepolytraj(configWaypoints,timeInterval,t);
%     % 计算机器人运动微分
%     stateDot = derivative(motionModel,state,[qd;qdDot]);
% end