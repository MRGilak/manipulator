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
    robot.setJointAngles([0; 0; -2*pi/3; 0; pi/3; 0]);
    % Configure link visualization
    robot.visual.linkLengths = [l1, l2, l3, l4, l5, l6];
    robot.visual.linkType = {'z', 'xy', 'xy', 'z', 'xy', 'z'};
    robot.visual.linkOffset = [0, 0, 0, l3, 0, l5];
    robot.visual.linkThetaOffset = [0, 0, pi/2, 0, -pi/2, 0];

    mass = 0.10*[10, 10, 10, 1, 1, 1];
    robot.setMassAndInertia(mass);

    robot.comOffset = [0, (l2)/2, 0, 0, 0, 0; ...
                        (l1)/2, 0, 0, (l4)/2, 0, 0; ...
                        0, 0, -(l3)/2, 0, -(l5)/2, (l6)/2];  % 3x6 matrix with COM vectors

    % End-effector options
    robot.visual.showEndEffector = true;              % Show/hide
    robot.visual.endEffectorType = 'gripper';         % 'gripper', 'tool', or 'sphere'
    robot.visual.endEffectorSize = 80;                % Size in mm
    robot.visual.endEffectorColor = [0.3 0.3 0.3];    % RGB color (dark gray)  

    %% Create and run simulation
    sim = Simulation(robot);
    sim.mode = 'task-space';
    
    sim.plotConfig.q = true;
    sim.plotConfig.qdot = true;

    %% Get initial end-effector pose
    T_init = robot.fk(robot.n);
    O_center = T_init(1:3, 4);
    R_d = T_init(1:3, 1:3);

    %% Circle trajectory parameters
    radius = 500;           % Circle radius (mm)
    T_total = 25;           % Period (seconds)
    
    %% Create trajectory function
    trajectoryFunc = @(t) circleTrajectory(t, T_total, R_d, O_center, radius);
    
    % Set trajectory tracking
    sim.trajectoryFunc = trajectoryFunc;
    sim.useTrajectory = true;

    %% Controller setup
    % Initial desired position
    qdes = robot.q;
    qdotdes = zeros(6, 1);
    qddotdes = zeros(6, 1);

    Lambda = 0.005 * diag([1000, 1000, 1000, 100, 100, 100]);
    K = 1000 * diag([1000, 1000, 1000, 100, 100, 100]);

    % Slotine controller
     controller = Controller(robot, 'Slotine', ...
         0.01, Lambda, K, qdes, qdotdes, qddotdes);
    
    sim.addController(controller);

    sim.run('headless', false, 'draw', true);
end

function [R, O] = circleTrajectory(t, T_total, R_d, O_center, radius)
    % Circular trajectory function
    angle = 2*pi*t/T_total;
    O = O_center + [radius*cos(angle); radius*sin(angle); 0];
    R = R_d;  % Keep orientation constant
end
