function circle_trajectory
    clc; close all;

    %% DH parameters
    l1 = 396; l2 = 324; l3 = 220; l4 = 123; l5 = 165; l6 = 50;

    a     = [0, l2, 0, 0, 0, 0];
    d     = [l1, 0, 0, l3+l4, 0, l5+l6];
    alpha = deg2rad([90, 0, -90, -90, 90, 0]);

    jointType = ['R','R','R','R','R','R'];

    %% Create robot
    robot = Manipulator(a, d, alpha, jointType);
    robot.setJointAngles([0; pi/2; -pi/2; 0; 0; 0]);
    robot.visual.linkLengths = [l1, l2, l3, l4, l5, l6];
    robot.visual.linkType = {'z', 'xy', 'xy', 'z', 'xy', 'z'};
    robot.visual.linkOffset = [0, 0, 0, l3, 0, l5];
    robot.visual.linkThetaOffset = [0, 0, pi/2, 0, -pi/2, 0];

    mass = 0.10*[10, 10, 10, 1, 1, 1];
    robot.setMassAndInertia(mass);

    robot.comOffset = [0, (l2)/2, 0, 0, 0, 0; ...
                        (l1)/2, 0, 0, (l4)/2, 0, 0; ...
                        0, 0, -(l3)/2, 0, -(l5)/2, (l6)/2];

    robot.visual.showEndEffector = true;
    robot.visual.endEffectorType = 'gripper';
    robot.visual.endEffectorSize = 80;
    robot.visual.endEffectorColor = [0.3 0.3 0.3];

    %% Get initial end-effector pose
    T_init = robot.fk(robot.n);
    O_center = T_init(1:3, 4);
    R_d = T_init(1:3, 1:3);

    %% Circle parameters
    radius = 150;
    T_total = 10;
    
    %% Create trajectory function
    trajectoryFunc = @(t) circleTrajectory(t, T_total, R_d, O_center, radius);
    
    %% Create controller (using existing joint-space controller)
    % PD controller with gravity compensation
    Kp = 1000 * eye(6);
    Kd = 100 * eye(6);
    qdes_init = robot.q;  % Initial desired position
    
    controller = Controller(robot, 'PD With Gravity Compensation', 0.01, Kp, Kd, qdes_init);
    
    %% Create simulation
    sim = Simulation(robot);
    sim.addController(controller);
    sim.mode = 'task-space';
    sim.dt = 0.01;
    
    % Set trajectory tracking
    sim.trajectoryFunc = trajectoryFunc;
    sim.useTrajectory = true;
    
    sim.run();
end

function [R, O] = circleTrajectory(t, T_total, R_d, O_center, radius)
    % Circular trajectory function
    angle = 2*pi*t/T_total;
    O = O_center + [radius*cos(angle); radius*sin(angle); 0];
    R = R_d;  % Keep orientation constant
end
