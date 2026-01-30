function main
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
    sim.mode = 'dynamic';
    
    sim.plotConfig.q = true;
    sim.plotConfig.qdot = true;
    % sim.plotConfig.debugInertia = true;
    % sim.plotConfig.debugSkewSymmetry = true;

    qdes = [0; 0; 0; pi/2; 0; -pi/2];
    qdotdes = zeros(6, 1);
    qddotdes = zeros(6, 1);

    Kp = diag([100, 100, 100, 100, 10, 100]);
    Kd = diag([100, 100, 100, 10, 10, 10]);

    % PD controller
    controller = Controller(robot, 'PD With Gravity Compensation', ...
        0.01, Kp, Kd, qdes);

    sim.addController(controller);

    sim.run();
end
