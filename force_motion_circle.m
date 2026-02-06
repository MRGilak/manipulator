function force_motion_circle
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
    sim.mode = 'constrained';
    
    sim.plotConfig.q = true;
    sim.plotConfig.qdot = true;
    % sim.plotConfig.u = true;  % Torque
    % sim.plotConfig.debugInertia = true;
    % sim.plotConfig.debugSkewSymmetry = true;
    
    % Auto-save results when simulation stops
    sim.plotConfig.saveResults = false;  % Set to true to auto-save
    sim.plotConfig.saveFilename = 'sim_results.mat';  % Output filename
    sim.plotConfig.saveExcel = false;  % Set to true to also save as Excel

    %% Circular trajectory constraint parameters
    % Circle parameters - 5cm radius at height 0cm (floor level)
    radius = 50;            % Circle radius: 5cm = 50mm
    center = [0; 500];      % Circle center: [0, 50, 0]cm = [0, 500, 0]mm in x-y
    z0 = 0;                 % Height: 0cm (floor level)
    omega_circle = 1;       % Angular velocity: 1 rad/sec
    T_total = 2*pi/omega_circle;  % Period for one complete revolution
    
    % Environment Jacobian: constrain end-effector z velocity to zero
    J_e = [0, 0, 1, 0, 0, 0];  % Only linear z motion constrained
    robot.setEnvironmentJacobian(J_e);

    %% Desired trajectory function (respects constraint)
    trajectoryFunc = @(t) circularConstrainedTrajectory(t, T_total, center, radius, z0);
    
    %% Controller setup
    % Get initial desired configuration from inverse kinematics
    [R_d, O_d, ~, ~] = trajectoryFunc(0);
    T_d = [R_d, O_d; 0 0 0 1];
    qdes = robot.ik(T_d, robot.q);
    qdotdes = zeros(6, 1);
    qddotdes = zeros(6, 1);

    % Desired constraint force: 2N upward from floor (in +z direction)
    F_des = [0; 0; 2000; 0; 0; 0];  % 2N = 2000mN (force in mN units)

    K = 0.1 * diag([1000, 1000, 1000, 100, 100, 100]);

    % Force-motion controller
    controller = Controller(robot, 'Force-motion', ...
        0.01, K, qdes, qdotdes, qddotdes, F_des);
    
    % Actuator saturation (optional)
    % controller.setSaturation(1e6, -1e6);  % Symmetric limits
    % controller.setSaturation([1e6; 1e6; 1e6; 1e5; 1e5; 1e5], ...
    %                          -[1e6; 1e6; 1e6; 1e5; 1e5; 1e5]);  % Per-joint limits

    sim.addController(controller);

    % Set up trajectory tracking callback to update desired values
    sim.trajectoryFunc = trajectoryFunc;
    sim.useTrajectory = true;

    sim.run('headless', false, 'draw', false);
    
    % To load and plot saved results, use:
    % Simulation.loadAndPlot('sim_results.mat');
    % To animate saved results:
    % Simulation.loadAndPlot('sim_results.mat', 'animate', true);
end

function [R, O, v, omega] = circularConstrainedTrajectory(t, T_total, center, radius, z0)
    % Angle and its derivative (counterclockwise at 1 rad/sec)
    theta = 2*pi*t/T_total;
    theta_dot = 2*pi/T_total;  % This equals 1 rad/sec when T_total = 2*pi
    
    % Position (circular motion in x-y plane)
    O = [center(1) + radius*cos(theta); 
         center(2) + radius*sin(theta); 
         z0];  % Constant height
    
    % Linear velocity
    v = [-radius*sin(theta)*theta_dot;
         radius*cos(theta)*theta_dot;
         0];  % vz = 0 satisfies constraint
    
    % Keep orientation constant
    R = [1, 0, 0; 0, 0, 1; 0, -1, 0];
    omega = [0; 0; 0];
end
