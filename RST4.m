robot = loadrobot('kinovaGen3','DataFormat','row','Gravity',[0 0 -9.81]);
currentRobotJConfig = homeConfiguration(robot);
numJoints = numel(currentRobotJConfig);
endEffector = "EndEffector_Link";
timeStep = 0.1; % seconds
toolSpeed = 0.1; % m/s

jointInit = currentRobotJConfig;
taskInit = getTransform(robot,jointInit,endEffector);

taskFinal = trvec2tform([0.4,0,0.6])*axang2tform([0 1 0 pi]);

distance = norm(tform2trvec(taskInit)-tform2trvec(taskFinal));

initTime = 0;
finalTime = (distance/toolSpeed) - initTime;
trajTimes = initTime:timeStep:finalTime;
timeInterval = [trajTimes(1); trajTimes(end)];

[taskWaypoints,taskVelocities] = transformtraj(taskInit,taskFinal,timeInterval,trajTimes); 

tsMotionModel = taskSpaceMotionModel('RigidBodyTree',robot,'EndEffectorName','EndEffector_Link');

tsMotionModel.Kp(1:3,1:3) = 0;
tsMotionModel.Kd(1:3,1:3) = 0;

q0 = currentRobotJConfig; 
qd0 = zeros(size(q0));

[tTask,stateTask] = ode15s(@(t,state) exampleHelperTimeBasedTaskInputs(tsMotionModel,timeInterval,taskInit,taskFinal,t,state),timeInterval,[q0; qd0]);

ik = inverseKinematics('RigidBodyTree',robot);
ik.SolverParameters.AllowRandomRestart = false;
weights = [1 1 1 1 1 1];

initialGuess = jointInit;
jointFinal = ik(endEffector,taskFinal,weights,initialGuess);

wrappedJointFinal = wrapToPi(jointFinal);

ctrlpoints = [jointInit',wrappedJointFinal'];
jointConfigArray = cubicpolytraj(ctrlpoints,timeInterval,trajTimes);

jsMotionModel = jointSpaceMotionModel('RigidBodyTree',robot,'MotionType','PDControl');

q0 = currentRobotJConfig; 
qd0 = zeros(size(q0));

[tJoint,stateJoint] = ode15s(@(t,state) exampleHelperTimeBasedJointInputs(jsMotionModel,timeInterval,jointConfigArray,t,state),timeInterval,[q0; qd0]);

show(robot,currentRobotJConfig,'PreservePlot',false,'Frames','off');
hold on
axis([-1 1 -1 1 -0.1 1.5]);

for i=1:length(trajTimes)
    % Current time 
    tNow= trajTimes(i);
    % Interpolate simulated joint positions to get configuration at current time
    configNow = interp1(tTask,stateTask(:,1:numJoints),tNow);
    poseNow = getTransform(robot,configNow,endEffector);
    show(robot,configNow,'PreservePlot',false,'Frames','off');
    taskSpaceMarker = plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'b.','MarkerSize',20);
    drawnow;
end

% Return to initial configuration
show(robot,currentRobotJConfig,'PreservePlot',false,'Frames','off');

for i=1:length(trajTimes)
    % Current time 
    tNow= trajTimes(i);
    % Interpolate simulated joint positions to get configuration at current time
    configNow = interp1(tJoint,stateJoint(:,1:numJoints),tNow);
    poseNow = getTransform(robot,configNow,endEffector);
    show(robot,configNow,'PreservePlot',false,'Frames','off');
    jointSpaceMarker = plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'r.','MarkerSize',20);
    drawnow;
end

% Add a legend and title
legend([taskSpaceMarker jointSpaceMarker], {'Defined in Task-Space', 'Defined in Joint-Space'});
title('Manipulator Trajectories')

function stateDot = exampleHelperTimeBasedTaskInputs(motionModel,timeInterval,initialTform,finalTform,t,state)

    [refPose,refVel] = transformtraj(initialTform,finalTform,timeInterval,t);
    stateDot = derivative(motionModel,state,refPose,refVel);

end

function stateDot = exampleHelperTimeBasedJointInputs(motionModel, timeInterval,configWaypoints,t,state)

    [qd,qdDot] = bsplinepolytraj(configWaypoints,timeInterval,t);
    stateDot = derivative(motionModel,state,[qd;qdDot]);

end

