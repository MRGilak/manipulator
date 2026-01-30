classdef Controller < handle
    properties
        type        % PD, Slotine, Robust, Adaptive
        robot       % the manipulator robot 
        dt          % control timestep

        % Desired values
        qdes
        qdotdes
        qddotdes

        % PD controller parameters
        Kp
        Kd

        % Slotine contrller parameters
        Lambda
        K
    end

    methods
        %% Constructor
        function obj = Controller(robot, type, dt, varargin)
            obj.robot = robot;
            obj.type = type;
            obj.dt = dt;

            obj.Kp = zeros(6, 6);
            obj.Kd = zeros(6, 6);

            if nargin > 3
                if obj.type == "PD"
                    obj.Kp = varargin{1};
                    obj.Kd = varargin{2};
                    obj.qdes = varargin{3};
                elseif obj.type == "PD With Gravity Compensation"
                    obj.Kp = varargin{1};
                    obj.Kd = varargin{2};
                    obj.qdes = varargin{3};
                elseif obj.type == "Slotine"
                    obj.Lambda = varargin{1};
                    obj.K = varargin{2};
                    obj.qdes = varargin{3};
                    obj.qdotdes = varargin{4};
                    obj.qddotdes = varargin{5};
                end
            end
        end

        function u = uNext(obj, varargin)
            switch obj.type 
                case 'Open-Loop'
                    u = zeros(6, 1);
                case 'Gravity Compensation'
                    u = obj.robot.gravityTorque;
                case 'PD'
                    u = obj.Kp * (obj.qdes - obj.robot.q) - obj.Kd * obj.robot.qdot;
                case 'PD With Gravity Compensation'
                    u = obj.robot.gravityTorque + obj.Kp * (obj.qdes - obj.robot.q) - obj.Kd * obj.robot.qdot;
                case 'Slotine'
                    v = obj.qdotdes - obj.Lambda*(obj.robot.q - obj.qdes);
                    vdot = obj.qddotdes - obj.Lambda*(obj.robot.qdot - obj.qdotdes);
                    s = obj.robot.qdot - v;

                    u = obj.robot.inertiaMatrix*vdot + obj.robot.coriolisCentrifugalMatrix*v + obj.robot.gravityTorque - obj.K*s;

                    fprintf("eigD = \n");
                    disp(eig(obj.robot.inertiaMatrix));
                    fprintf("v = \n");
                    disp(v);
                    fprintf("vdot = \n");
                    disp(vdot);
                    fprintf("s = \n");
                    disp(s);
                    fprintf("u = \n");
                    disp(u);
            end
        end
    end
end