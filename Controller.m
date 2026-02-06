classdef Controller < handle
    properties
        type        % PD, Slotine, Robust, Adaptive, Force-motion
        robot       % the manipulator robot 
        dt          % control timestep

        % Desired values (joint space)
        qdes
        qdotdes
        qddotdes

        % PD controller parameters
        Kp
        Kd

        % Slotine contrller parameters
        Lambda
        K
        
        % Force-motion controller parameters
        F_des       % Desired constraint force (6x1 vector)
        
        % Actuator saturation
        useSaturation   % Boolean flag for saturation
        uMax            % Maximum torque (Nx1 vector or scalar)
        uMin            % Minimum torque (Nx1 vector or scalar)
    end

    methods
        %% Constructor
        function obj = Controller(robot, type, dt, varargin)
            obj.robot = robot;
            obj.type = type;
            obj.dt = dt;

            obj.Kp = zeros(6, 6);
            obj.Kd = zeros(6, 6);
            
            % Default: no saturation
            obj.useSaturation = false;
            obj.uMax = inf(robot.n, 1);
            obj.uMin = -inf(robot.n, 1);

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
                elseif obj.type == "Force-motion"
                    obj.K = varargin{1};
                    obj.qdes = varargin{2};
                    obj.qdotdes = varargin{3};
                    obj.qddotdes = varargin{4};
                    obj.F_des = varargin{5};
                end
            end
        end

        function u = uNext(obj, varargin)
            switch obj.type 
                case 'Open-Loop'
                    u = zeros(6, 1);
                case 'Gravity Compensation'
                    u = obj.robot.gravityTorque();
                case 'PD'
                    u = obj.Kp * (obj.qdes - obj.robot.q) - obj.Kd * obj.robot.qdot;
                case 'PD With Gravity Compensation'
                    u = obj.robot.gravityTorque() + obj.Kp * (obj.qdes - obj.robot.q) - obj.Kd * obj.robot.qdot;
                case 'Slotine'
                    v = obj.qdotdes - obj.Lambda*(obj.robot.q - obj.qdes);
                    vdot = obj.qddotdes - obj.Lambda*(obj.robot.qdot - obj.qdotdes);
                    s = obj.robot.qdot - v;

                    u = obj.robot.inertiaMatrix()*vdot + obj.robot.coriolisCentrifugalMatrix(obj.dt)*v + obj.robot.gravityTorque() - obj.K*s;
                case 'Force-motion'
                    D = obj.robot.inertiaMatrix();
                    C = obj.robot.coriolisCentrifugalMatrix(obj.dt);
                    G = obj.robot.gravityTorque();
                    J = obj.robot.jacobian(obj.robot.n);
                    
                    % Ensure all desired values have correct dimensions
                    if isempty(obj.qddotdes) || length(obj.qddotdes) ~= obj.robot.n
                        obj.qddotdes = zeros(obj.robot.n, 1);
                    end
                    if isempty(obj.qdotdes) || length(obj.qdotdes) ~= obj.robot.n
                        obj.qdotdes = zeros(obj.robot.n, 1);
                    end
                    if isempty(obj.F_des) || length(obj.F_des) ~= 6
                        obj.F_des = zeros(6, 1);
                    end
                    
                    qtildedot = obj.robot.qdot - obj.qdotdes;
                    
                    u = D*obj.qddotdes + C*obj.qdotdes + G + J'*obj.F_des - obj.K*qtildedot;
            end
            
            % Apply actuator saturation if enabled
            if obj.useSaturation
                u = max(min(u, obj.uMax), obj.uMin);
            end
        end
        
        function setSaturation(obj, uMax, uMin)
            obj.useSaturation = true;
            if isscalar(uMax)
                obj.uMax = uMax * ones(obj.robot.n, 1);
            else
                obj.uMax = uMax(:);
            end
            if isscalar(uMin)
                obj.uMin = uMin * ones(obj.robot.n, 1);
            else
                obj.uMin = uMin(:);
            end
        end
    end
end