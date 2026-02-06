%% Note: This function is still work in progress and is not completed yet!

function force_motion_circle
    clc; close all;
    warning('off', 'MATLAB:logm:nonPosRealEig');

    %% DH parameters
    l1 = 396; l2 = 324; l3 = 220; l4 = 123; l5 = 165; l6 = 50;

    a     = [0, l2, 0, 0, 0, 0];
    d     = [l1, 0, 0, l3+l4, 0, l5+l6];
    alpha = deg2rad([90, 0, -90, -90, 90, 0]);

    jointType = ['R','R','R','R','R','R'];

    %% Create robot
    robot = Manipulator(a, d, alpha, jointType);

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
    
    %% Circular trajectory constraint parameters
    radius = 50;            % 50mm
    center = [0; 500];      
    z0 = 0;                 % Height
    omega_circle = 0.5;     % Angular velocity: 1 rad/sec
    T_total = 2*pi/omega_circle;  % Period for one complete revolution
    
    % Desired trajectory function
    trajectoryFunc = @(t) circularConstrainedTrajectory(t, T_total, center, radius, z0);
    
    %% Initialize robot at starting position using IK
    % Get initial desired pose from trajectory at t=0
    [R_d, O_d, ~, ~] = trajectoryFunc(0);
    T_d = [R_d, O_d; 0 0 0 1];
    
    % Solve IK to find joint angles that place EF at starting position
    q_init_guess = [0; pi/2; -pi/2; 0; 0; 0];
    q_start = robot.ik(T_d, q_init_guess);
    
    % Set robot to starting configuration
    robot.setJointAngles(q_start);
    
    %% Set environment constraint
    % Environment Jacobian: constrain end-effector z velocity to zero
    J_e = [0, 0, 1, 0, 0, 0];  % Only linear z motion constrained
    robot.setEnvironmentJacobian(J_e);
    
    %% Create simulation
    sim = Simulation(robot);
    sim.mode = 'constrained';
    
    sim.plotConfig.q = true;
    sim.plotConfig.qdot = true;
    
    %% Controller setup
    qdes = q_start;  % Start at the IK solution
    qdotdes = zeros(6, 1);
    qddotdes = zeros(6, 1);

    % Desired constraint force: 2N upward from floor 
    F_des = [0; 0; 2000; 0; 0; 0];  % 2N = 2000mN

    K = 1e2 * diag([1000, 1000, 1000, 100, 100, 100]); 

    % Force-motion controller
    controller = Controller(robot, 'Force-motion', ...
        0.01, K, qdes, qdotdes, qddotdes, F_des);

    sim.addController(controller);

    % Set up trajectory tracking callback to update desired values
    sim.trajectoryFunc = trajectoryFunc;
    sim.useTrajectory = true;

    sim.run('headless', false, 'draw', false);
end

function [R, O, v, omega] = circularConstrainedTrajectory(t, T_total, center, radius, z0)
    % Angle and its derivative (counterclockwise at 1 rad/sec)
    theta = 2*pi*t/T_total;
    theta_dot = 2*pi/T_total;
    
    % Position (circular motion in x-y plane)
    O = [center(1) + radius*cos(theta); 
         center(2) + radius*sin(theta); 
         z0];  % Constant height
    
    % Linear velocity
    v = [-radius*sin(theta)*theta_dot;
         radius*cos(theta)*theta_dot;
         0];  % vz = 0 satisfies constraint
    
    % End-effector orientation pointing downward 
    R = [1,  0,  0; 
         0, -1,  0; 
         0,  0, -1];
    omega = [0; 0; 0];
end
