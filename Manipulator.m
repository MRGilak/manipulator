classdef Manipulator < handle
    properties
        % DH parameters
        a
        d
        alpha
        jointType   % 'R' or 'P' (revolute or prismatic)
        n

        % State
        q
        qdot

        % State history
        q_his
        qdot_his
        qddot_his
        u_his

        % Base pose
        baseT

        % Visualization
        visual
        graphics
        
        % Center of mass
        comOffset  % 3xn matrix: vector from frame i to COM of link i
        
        % Dynamic properties
        mass       % 1xn vector: mass of each link
        inertia    % 3x3xn array: inertia matrix of each link in its frame
        g0         % Gravity constant (default: 9810 mm/s^2)
        J_prev     % Previous COM Jacobian for computing J_dot
        dt_prev    % Previous time step for J_dot computation
    end

    methods
        %% Constructor
        function obj = Manipulator(a,d,alpha,jointType,varargin)
            obj.a = a(:)';
            obj.d = d(:)';
            obj.alpha = alpha(:)';
            obj.jointType = jointType(:)';
            obj.n = length(a);

            obj.q = zeros(obj.n,1);
            obj.qdot = zeros(obj.n,1);

            obj.q_his = zeros(obj.n,1);
            obj.qdot_his = zeros(obj.n,1);
            obj.qddot_his = zeros(obj.n,1);
            obj.u_his = zeros(obj.n,1);

            % Base pose
            if nargin > 4
                obj.baseT = varargin{1};
            else
                obj.baseT = eye(4);
            end

            % Visual defaults
            obj.visual = struct();
            obj.visual.linkRadius     = [50, 50, 50, 10, 10, 10];
            obj.visual.linkResolution = 20;
            
            % Link lengths
            obj.visual.linkLengths = sqrt(obj.a.^2 + obj.d.^2);  % Default estimate
            
            % Link offset: distance from frame (i-1) origin where link starts
            obj.visual.linkOffset = zeros(1, obj.n);
            
            % Theta offset : final angle = q(i) + linkThetaOffset(i)
            obj.visual.linkThetaOffset = zeros(1, obj.n);
            
            % Link type: 'z' = extends along z-axis of frame (i-1)
            %            'xy' = extends in x-y plane of frame (i-1), rotated by q(i)
            obj.visual.linkType = cell(1, obj.n);
            for i = 1:obj.n
                % Default: 'z' if d dominates, 'xy' if a dominates
                if obj.d(i) >= obj.a(i)
                    obj.visual.linkType{i} = 'z';
                else
                    obj.visual.linkType{i} = 'xy';
                end
            end
            
            obj.visual.linkColor = lines(obj.n);
            
            % Frame visualization
            obj.visual.frameSize = 100;  % Length of frame axes
            obj.visual.showFrames = true;
            obj.visual.showLabels = true;
            
            % End-effector visualization
            obj.visual.showEndEffector = true;
            obj.visual.endEffectorSize = 80;  % Size of end-effector
            obj.visual.endEffectorType = 'gripper';  % 'gripper', 'tool', 'sphere'
            obj.visual.endEffectorColor = [0.3 0.3 0.3];  % Dark gray
            
            % Center of mass offsets (default: at frame origin)
            obj.comOffset = zeros(3, obj.n);
            
            % Dynamic properties (defaults)
            obj.mass = ones(1, obj.n);  % Unit mass
            obj.inertia = zeros(3, 3, obj.n);
            for i = 1:obj.n
                obj.inertia(:,:,i) = eye(3);  % Unit inertia
            end
            obj.g0 = 9810;  % mm/s^2

            obj.calculateInertia();
            
            % Jacobian derivative computation
            obj.J_prev = [];

            obj.graphics = struct();
        end

        %% Set joint angles
        function setJointAngles(obj,q)
            obj.q = q(:);
        end

        function setMassAndInertia(obj, mass, varargin)
            Mass = mass(:);
            obj.mass = Mass';
            
            if nargin > 2
                Inertia = varargin{1}(:);
                obj.inertia = Inertia';
            end
        end

        function obj = calculateInertia(obj)
            for i = 1:obj.n
                obj.inertia(:,:,i) = [((1/12)*obj.mass(i)*(obj.visual.linkLengths(i))^2) + ...
                                        ((1/4)*obj.mass(i)*(obj.visual.linkRadius(i))^2), 0, 0; ...
                                        0, ((1/12)*obj.mass(i)*(obj.visual.linkLengths(i))^2) + ...
                                        ((1/4)*obj.mass(i)*(obj.visual.linkRadius(i))^2), 0;
                                        0, 0, ((1/12)*obj.mass(i)*(obj.visual.linkLengths(i))^2)];
            end
        end

        % Denavit-Hartenberg
        function H = dh(obj, theta, d, a, alpha)
            M_theta = [ cos(theta) -sin(theta)  0  0;
                        sin(theta)  cos(theta)  0  0;
                        0           0           1  0;
                        0           0           0  1 ];
        
            M_d = [ 1 0 0 0;
                    0 1 0 0;
                    0 0 1 d;
                    0 0 0 1 ];
        
            M_a = [ 1 0 0 a;
                    0 1 0 0;
                    0 0 1 0;
                    0 0 0 1 ];
        
            M_alpha = [ 1 0           0            0;
                        0 cos(alpha) -sin(alpha)   0;
                        0 sin(alpha)  cos(alpha)   0;
                        0 0           0            1 ];
        
            H = M_theta * M_d * M_a * M_alpha;
        end

        %% DH transform for joint i
        function Ti = dhTransform(obj,i)
            if obj.jointType(i) == 'R'
                theta = obj.q(i);
                d_i   = obj.d(i);
            else
                theta = 0;
                d_i   = obj.q(i);
            end

            a_i = obj.a(i);
            alpha_i = obj.alpha(i);

            Ti = obj.dh(theta,d_i,a_i,alpha_i);
        end

        %% Forward kinematics to frame idx
        function T = fk(obj,frameIdx)
            T = obj.baseT;
            for i = 1:frameIdx
                T = T * obj.dhTransform(i);
            end
        end

        %% Angular Jacobian
        function Jw = jacobianOmega(obj,frameIdx)
            % frameIdx: index of the frame (default is end-effector)
            if nargin < 2
                frameIdx = obj.n;
            end
            
            k = [0;0;1];
            Jw = zeros(3,obj.n);

            T = obj.baseT;
            R = T(1:3,1:3);

            for i = 1:obj.n
                if i <= frameIdx && obj.jointType(i) == 'R'
                    Jw(:,i) = R*k;
                end

                if i < frameIdx
                    T = T * obj.dhTransform(i);
                    R = T(1:3,1:3);
                end
            end
        end

        %% Linear Jacobian
        function Jv = jacobianLinear(obj,frameIdx)
            % frameIdx: index of the frame (default is end-effector)
            if nargin < 2
                frameIdx = obj.n;
            end

            k = [0;0;1];
            Jv = zeros(3,obj.n);

            T = obj.baseT;
            p = T(1:3,4);
            R = T(1:3,1:3);

            p_n = obj.fk(frameIdx);
            p_n = p_n(1:3,4);

            for i = 1:obj.n
                if i <= frameIdx
                    z = R*k;
                    if obj.jointType(i) == 'R'
                        Jv(:,i) = cross(z,p_n - p);
                    else
                        Jv(:,i) = z;
                    end
                end

                if i < frameIdx
                    T = T * obj.dhTransform(i);
                    R = T(1:3,1:3);
                    p = T(1:3,4);
                end
            end
        end

        %% Full Jacobian combining linear and angular velocities
        function J = jacobian(obj, frameIdx)
            % Get full 6xn Jacobian matrix [Jv; Jw]
            % frameIdx: index of the frame (default is end-effector)
            if nargin < 2
                frameIdx = obj.n;
            end
            Jv = obj.jacobianLinear(frameIdx);
            Jw = obj.jacobianOmega(frameIdx);
            J = [Jv; Jw];
        end
        
        %% COM Jacobian for center of mass of a link
        function J_C = jacobianCOM(obj, linkIdx)
            % Compute Jacobian for center of mass of link
            % J_C = [I, S(r_CD); 0, I] * J_D
            % where r_CD is vector from COM to frame on that link
            % and J_D is the geometric Jacobian at that frame
            if nargin < 2
                linkIdx = obj.n;
            end
            
            % Get geometric Jacobian at frame linkIdx
            J_D = obj.jacobian(linkIdx);
            
            % Get COM offset for this link
            r_CD = obj.comOffset(:, linkIdx);
            
            % Build transformation matrix
            I3 = eye(3);
            S_r = obj.skew(r_CD);
            T_CD = [I3, S_r; zeros(3,3), I3];
            
            % Compute COM Jacobian
            J_C = T_CD * J_D;
        end
        
        %% Mass-Inertia matrix M (6n x 6n)
        function M = massMatrix(obj)
            % Build 6n x 6n mass-inertia matrix
            n6 = 6 * obj.n;
            M = zeros(n6, n6);
            
            for i = 1:obj.n
                % Top-left quarter: mass * identity
                idx = (i-1)*3 + (1:3);
                M(idx, idx) = obj.mass(i) * eye(3);
                
                % Bottom-right quarter: inertia matrix
                idx_rot = 3*obj.n + (i-1)*3 + (1:3);
                M(idx_rot, idx_rot) = obj.inertia(:,:,i);
            end
        end
        
        %% Coriolis matrix S (6n x 6n)
        function S = coriolisMatrix(obj)
            % Build 6n x 6n coriolis/centrifugal matrix
            n6 = 6 * obj.n;
            S = zeros(n6, n6);
            
            for i = 1:obj.n
                % Get angular velocity for link i
                Jw = obj.jacobianOmega(i);
                omega_i = Jw * obj.qdot;
                
                % Compute I_i * omega_i
                I_omega = obj.inertia(:,:,i) * omega_i;
                
                % Bottom-right quarter: -S(I_i * omega_i)
                idx = 3*obj.n + (i-1)*3 + (1:3);
                S(idx, idx) = -obj.skew(I_omega);
            end
        end
        
        %% Gravity vector g (6n x 1)
        function g = gravityVector(obj)
            % Build 6n x 1 gravity vector
            n6 = 6 * obj.n;
            g = zeros(n6, 1);
            
            for i = 1:obj.n
                % Top half: gravity force
                idx = (i-1)*3 + (1:3);
                g(idx) = [0; 0; obj.g0] * obj.mass(i);
            end
        end
        
        %% Stacked COM Jacobian for all links (6n x n)
        function J_all = jacobianCOM_all(obj)
            J_all = zeros(6*obj.n, obj.n);
            
            for i = 1:obj.n
                % Get COM Jacobian for link i
                J_C = obj.jacobianCOM(i);
                
                % Split into linear and angular 
                J_v = J_C(1:3, :);
                J_omega = J_C(4:6, :);
                
                % Top half: stack linear velocity Jacobians
                idx_v = (i-1)*3 + (1:3);
                J_all(idx_v, :) = J_v;
                
                % Get rotation matrix from base to frame i
                T_i = obj.fk(i);
                R_i = T_i(1:3, 1:3);
                
                idx_omega = 3*obj.n + (i-1)*3 + (1:3);
                J_all(idx_omega, :) = R_i' * J_omega;  % R^(-1) = R^T for rotation matrix
            end
        end
        
        %% Time derivative of stacked COM Jacobian (6n x n)
        function J_dot = jacobianCOM_dot(obj, dt)
            % Compute numerical derivative of COM Jacobian
            
            J_current = obj.jacobianCOM_all();
            
            if isempty(obj.J_prev) || size(obj.J_prev,1) ~= size(J_current,1)
                % First call or dimension mismatch - initialize
                J_dot = zeros(size(J_current));
                obj.J_prev = J_current;
            else
                % Compute finite difference
                J_dot = (J_current - obj.J_prev) / dt;
                obj.J_prev = J_current;
            end
            
            obj.dt_prev = dt;
        end
        
        %% Inertia matrix D (n x n)
        function D = inertiaMatrix(obj)
            % Compute D = J^T * M * J
            J = obj.jacobianCOM_all();
            M = obj.massMatrix();
            D = J' * M * J;
        end

        %% Inertia matrix Derivative (n x n)
        function D_dot = inertiaMatrixDot(obj, dt)
            % Compute time derivative of inertia matrix D
            
            J = obj.jacobianCOM_all();
            M = obj.massMatrix();
            J_dot = obj.jacobianCOM_dot(dt);

            D_dot = J_dot' * M * J + J' * M * J_dot;
        end
        
        %% Coriolis/Centrifugal matrix C (n x n)
        function C = coriolisCentrifugalMatrix(obj, dt)
            % Compute C = J^T * M * J_dot + J^T * S * J
            if nargin < 2
                dt = obj.dt_prev;
            end
            
            J = obj.jacobianCOM_all();
            M = obj.massMatrix();
            S = obj.coriolisMatrix();
            J_dot = obj.jacobianCOM_dot(dt);
            
            C = J' * M * J_dot + J' * S * J;
        end
        
        %% Gravity torque vector G (n x 1)
        function G = gravityTorque(obj)
            % Compute G = J^T * g
            J = obj.jacobianCOM_all();
            g = obj.gravityVector();
            G = J' * g;
        end

        %% Skew-symmetric matrix
        function S = skew(~, w)
            % Create skew-symmetric matrix from vector w
            S = [0    -w(3)  w(2);
                 w(3)  0    -w(1);
                -w(2)  w(1)  0   ];
        end
        
        %% Update frame position from linear velocity
        function d_new = updatePosition(obj, d, v, dt)
            % Integrate: d_dot = v
            d_new = d + v * dt;
        end
        
        %% Update frame rotation from angular velocity
        function R_new = updateRotation(obj, R, omega, dt)
            % Integrate: R_dot = S(omega) * R
            S_omega = obj.skew(omega);
            R_new = expm(S_omega * dt) * R;
        end
        
        %% Update transformation matrix from velocity
        function T_new = updateTransform(obj, T, v, omega, dt)
            % Extract current position and rotation
            d = T(1:3, 4);
            R = T(1:3, 1:3);
            
            % Update position and rotation
            d_new = obj.updatePosition(d, v, dt);
            R_new = obj.updateRotation(R, omega, dt);
            
            % Construct new transformation matrix
            T_new = [R_new, d_new; 0 0 0 1];
        end
        
        %% Integrate joint velocities to update joint positions
        function integrateJointVelocities(obj, dt)
            % Update q from qdot
            obj.q = obj.q + obj.qdot * dt;
        end
        
        %% Debug: Check if D is positive definite
        function [isPD, eigVals] = checkInertiaMatrixPD(obj)
            D = obj.inertiaMatrix();
            eigVals = eig(D);
            isPD = all(eigVals > 0);
            if isPD
                fprintf('D is positive definite. Eigenvalues: %s\n', mat2str(eigVals, 4));
            else
                fprintf('D is NOT positive definite. Eigenvalues: %s\n', mat2str(eigVals, 4));
            end
        end
        
        %% Debug: Check if Ddot - 2C is skew-symmetric
        function [isSkew, err] = checkSkewSymmetry(obj, dt)
            J = obj.jacobianCOM_all();
            M = obj.massMatrix();
            S_mat = obj.coriolisMatrix();
            J_dot = obj.jacobianCOM_dot(dt);
            
            D_dot = obj.inertiaMatrixDot(dt);
            C = obj.coriolisCentrifugalMatrix(dt);
            
            % Check individual components
            term1 = J_dot' * M * J - J' * M * J_dot;
            term2 = -2 * J' * S_mat * J;
            
            err_M = norm(M - M', 'fro');
            err_S = norm(S_mat + S_mat', 'fro');
            err_term1 = norm(term1 + term1', 'fro');
            err_term2 = norm(term2 + term2', 'fro');
            
            S = D_dot - 2*C;
            err = norm(S + S', 'fro');
            isSkew = err < 1e-6;
            
            fprintf('M symmetry error: %.2e\n', err_M);
            fprintf('S skew-symmetry error: %.2e\n', err_S);
            fprintf('Term1 (J_dot''*M*J - J''*M*J_dot) skew error: %.2e\n', err_term1);
            fprintf('Term2 (-2*J''*S*J) skew error: %.2e\n', err_term2);
            if isSkew
                fprintf('Ddot - 2C is skew-symmetric. Norm: %.2e\n', err);
            else
                fprintf('Ddot - 2C is NOT skew-symmetric. Norm: %.2e\n', err);
            end
        end

        %% Draw robot
        function draw(obj, ax)
            hold(ax,'on');
            % axis(ax,'equal');
            axis(ax,'vis3d');
            grid(ax,'on');
            view(ax,3);
        
            xlabel(ax,'X');
            ylabel(ax,'Y');
            zlabel(ax,'Z');

            xlim(ax,[-750 750]);
            ylim(ax,[-750 750]);
            zlim(ax,[0 1600]);
        
            obj.graphics.links = struct([]);
            obj.graphics.frames = struct([]);
            obj.graphics.labels = struct([]);
        
            % Draw each link from frame (i-1)
            for i = 1:obj.n
                % Get transform of frame (i-1)
                T = obj.fk(i-1);  % Frame i-1 (0 = base)
                
                % Origin of frame (i-1)
                p_frame = T(1:3,4);
                
                % Calculate link start and end based on type
                linkType = obj.visual.linkType{i};
                L = obj.visual.linkLengths(i);
                offset = obj.visual.linkOffset(i);
                
                if strcmpi(linkType, 'z')
                    % Case 1: Link extends along z-axis of frame (i-1)
                    z_axis = T(1:3,3);
                    p_start = p_frame + offset * z_axis;  % Start offset along z
                    p_end = p_start + L * z_axis;
                else  % 'xy'
                    % Case 2: Link in x-y plane, rotated by q(i) + offset from x-axis
                    x_axis = T(1:3,1);
                    y_axis = T(1:3,2);
                    z_axis = T(1:3,3);
                    % Start position offset along z-axis
                    p_start = p_frame + offset * z_axis;
                    % Direction in x-y plane: cos(theta)*x + sin(theta)*y
                    theta = obj.q(i) + obj.visual.linkThetaOffset(i);
                    direction = cos(theta) * x_axis + sin(theta) * y_axis;
                    p_end = p_start + L * direction;
                end
                
                p_prev = p_start;
                p_curr = p_end;
        
                % Link vector and length
                v = p_curr - p_prev;
                L = norm(v);
        
                if L < eps
                    continue;
                end
        
                % Create cylinder along Z
                [X,Y,Z] = cylinder(obj.visual.linkRadius(i), obj.visual.linkResolution);
                Z = Z * L;
        
                % Rotation: Z-axis to link direction
                z0 = [0;0;1];
                R = vrrotvec2mat(vrrotvec(z0, v / L));
        
                P = R * [X(:)'; Y(:)'; Z(:)'];
                Xr = reshape(P(1,:), size(X)) + p_prev(1);
                Yr = reshape(P(2,:), size(Y)) + p_prev(2);
                Zr = reshape(P(3,:), size(Z)) + p_prev(3);
        
                h = surf(ax, Xr, Yr, Zr, ...
                    'FaceColor', obj.visual.linkColor(i,:), ...
                    'EdgeColor','none', ...
                    'FaceLighting','gouraud', ...
                    'HandleVisibility','off');  % Don't show in legend
        
                obj.graphics.links(i).surf = h;
            end
        
            % Draw coordinate frames
            if obj.visual.showFrames
                obj.drawFrames(ax);
            end
            
            % Draw labels
            if obj.visual.showLabels
                obj.drawLabels(ax);
            end            
            % Draw end-effector
            if obj.visual.showEndEffector
                obj.drawEndEffector(ax);
            end        
            camlight(ax,'headlight');
            material(ax, 'dull');
        end

        %% Draw coordinate frames
        function drawFrames(obj, ax)
            frameSize = obj.visual.frameSize;
            
            % Draw base frame (frame 0)
            T = obj.baseT;
            handles = obj.drawSingleFrame(ax, T, frameSize);
            obj.graphics.frames(1).x = handles.x;
            obj.graphics.frames(1).y = handles.y;
            obj.graphics.frames(1).z = handles.z;
            
            % Draw all joint frames
            for i = 1:obj.n
                T = obj.fk(i);
                handles = obj.drawSingleFrame(ax, T, frameSize);
                obj.graphics.frames(i+1).x = handles.x;
                obj.graphics.frames(i+1).y = handles.y;
                obj.graphics.frames(i+1).z = handles.z;
            end
        end
        
        %% Draw single coordinate frame (RGB = XYZ)
        function handles = drawSingleFrame(obj, ax, T, frameSize)
            p = T(1:3,4);
            x_axis = T(1:3,1);
            y_axis = T(1:3,2);
            z_axis = T(1:3,3);
            
            % X-axis = RED
            handles.x = quiver3(ax, p(1), p(2), p(3), x_axis(1)*frameSize, x_axis(2)*frameSize, x_axis(3)*frameSize, ...
                'r', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'AutoScale', 'off', 'HandleVisibility', 'off');
            
            % Y-axis = GREEN
            handles.y = quiver3(ax, p(1), p(2), p(3), y_axis(1)*frameSize, y_axis(2)*frameSize, y_axis(3)*frameSize, ...
                'g', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'AutoScale', 'off', 'HandleVisibility', 'off');
            
            % Z-axis = BLUE
            handles.z = quiver3(ax, p(1), p(2), p(3), z_axis(1)*frameSize, z_axis(2)*frameSize, z_axis(3)*frameSize, ...
                'b', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'AutoScale', 'off', 'HandleVisibility', 'off');
        end
        
        %% Draw labels for frames and add legend for links
        function drawLabels(obj, ax)
            labelOffset = obj.visual.frameSize * 0.3;  % Offset for visibility
            
            % Label base frame
            T = obj.baseT;
            p = T(1:3,4);
            obj.graphics.labels(1).text = text(ax, p(1)+labelOffset, p(2)+labelOffset, p(3)+labelOffset, ...
                'F0', 'FontSize', 10, 'FontWeight', 'bold', ...
                'Color', 'k', 'BackgroundColor', [1 1 1 0.8]);
            
            % Label all frames with offset to handle coincident frames
            for i = 1:obj.n
                T = obj.fk(i);
                p = T(1:3,4);
                % Stagger labels slightly for coincident frames
                offset = labelOffset * (1 + 0.3*mod(i,3));
                obj.graphics.labels(i+1).text = text(ax, p(1)+offset, p(2)+offset, p(3)+offset, ...
                    sprintf('F%d', i), 'FontSize', 10, 'FontWeight', 'bold', ...
                    'Color', 'k', 'BackgroundColor', [1 1 1 0.8]);
            end
            
            % Create legend for links using invisible patch objects
            for i = 1:obj.n
                patch(ax, 'XData', NaN, 'YData', NaN, 'ZData', NaN, ...
                    'FaceColor', obj.visual.linkColor(i,:), ...
                    'EdgeColor', 'none', ...
                    'DisplayName', sprintf('Link %d', i));
            end
            
            legend(ax, 'Location', 'northeastoutside');
        end
        
        %% Draw end-effector
        function drawEndEffector(obj, ax)
            % Get end-effector frame (last frame)
            T_ee = obj.fk(obj.n);
            p = T_ee(1:3,4);
            x_axis = T_ee(1:3,1);
            y_axis = T_ee(1:3,2);
            z_axis = T_ee(1:3,3);
            
            size = obj.visual.endEffectorSize;
            color = obj.visual.endEffectorColor;
            
            switch obj.visual.endEffectorType
                case 'gripper'
                    % Draw simple parallel gripper
                    obj.drawGripper(ax, p, x_axis, y_axis, z_axis, size, color);
                case 'tool'
                    % Draw simple tool (cone)
                    obj.drawTool(ax, p, x_axis, y_axis, z_axis, size, color);
                case 'sphere'
                    % Draw sphere
                    obj.drawSphere(ax, p, size, color);
            end
        end
        
        %% Draw gripper end-effector
        function drawGripper(obj, ax, p, x_axis, y_axis, z_axis, size, color)
            % Palm (base plate)
            palmLength = size * 0.4;
            palmWidth = size * 0.6;
            palmThick = size * 0.15;
            
            % Create palm as a box
            [X, Y, Z] = obj.createBox(palmLength, palmWidth, palmThick);
            
            % Transform to end-effector frame
            R = [x_axis, y_axis, z_axis];
            for i = 1:numel(X)
                pt = R * [X(i); Y(i); Z(i)] + p;
                X(i) = pt(1); Y(i) = pt(2); Z(i) = pt(3);
            end
            
            obj.graphics.endEffector.palm = surf(ax, X, Y, Z, ...
                'FaceColor', color, 'EdgeColor', 'none', ...
                'FaceLighting', 'gouraud', 'HandleVisibility', 'off');
            
            % Fingers (two parallel jaws)
            fingerLength = size * 0.8;
            fingerWidth = size * 0.15;
            fingerThick = size * 0.1;
            fingerGap = size * 0.3;
            
            % Finger 1 (positive y side)
            [X1, Y1, Z1] = obj.createBox(fingerLength, fingerWidth, fingerThick);
            offset1 = p + R * [palmLength/2; fingerGap/2 + fingerWidth/2; 0];
            for i = 1:numel(X1)
                pt = R * [X1(i); Y1(i); Z1(i)] + offset1;
                X1(i) = pt(1); Y1(i) = pt(2); Z1(i) = pt(3);
            end
            
            obj.graphics.endEffector.finger1 = surf(ax, X1, Y1, Z1, ...
                'FaceColor', color * 1.2, 'EdgeColor', 'none', ...
                'FaceLighting', 'gouraud', 'HandleVisibility', 'off');
            
            % Finger 2 (negative y side)
            [X2, Y2, Z2] = obj.createBox(fingerLength, fingerWidth, fingerThick);
            offset2 = p + R * [palmLength/2; -fingerGap/2 - fingerWidth/2; 0];
            for i = 1:numel(X2)
                pt = R * [X2(i); Y2(i); Z2(i)] + offset2;
                X2(i) = pt(1); Y2(i) = pt(2); Z2(i) = pt(3);
            end
            
            obj.graphics.endEffector.finger2 = surf(ax, X2, Y2, Z2, ...
                'FaceColor', color * 1.2, 'EdgeColor', 'none', ...
                'FaceLighting', 'gouraud', 'HandleVisibility', 'off');
        end
        
        %% Draw tool end-effector
        function drawTool(obj, ax, p, x_axis, y_axis, z_axis, size, color)
            % Simple cone tool pointing along x-axis
            baseRadius = size * 0.3;
            tipLength = size;
            
            [X, Y, Z] = cylinder([baseRadius, 0], 20);
            Z = Z * tipLength;
            
            % Rotate to align with x-axis
            R = [x_axis, y_axis, z_axis];
            for i = 1:numel(X)
                % Original is along z, rotate to x
                pt = R * [Z(i); X(i); Y(i)] + p;
                X(i) = pt(1); Y(i) = pt(2); Z(i) = pt(3);
            end
            
            obj.graphics.endEffector.tool = surf(ax, X, Y, Z, ...
                'FaceColor', color, 'EdgeColor', 'none', ...
                'FaceLighting', 'gouraud', 'HandleVisibility', 'off');
        end
        
        %% Draw sphere end-effector
        function drawSphere(obj, ax, p, size, color)
            [X, Y, Z] = sphere(20);
            X = X * size * 0.5 + p(1);
            Y = Y * size * 0.5 + p(2);
            Z = Z * size * 0.5 + p(3);
            
            obj.graphics.endEffector.sphere = surf(ax, X, Y, Z, ...
                'FaceColor', color, 'EdgeColor', 'none', ...
                'FaceLighting', 'gouraud', 'HandleVisibility', 'off');
        end
        
        %% Helper: Create box vertices
        function [X, Y, Z] = createBox(~, length, width, height)
            % Create a box centered at origin
            x = [-1 1 1 -1 -1 1 1 -1] * length/2;
            y = [-1 -1 1 1 -1 -1 1 1] * width/2;
            z = [-1 -1 -1 -1 1 1 1 1] * height/2;
            
            % Define faces
            faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 3 4 8 7; 1 4 8 5; 2 3 7 6];
            
            X = zeros(4, 6);
            Y = zeros(4, 6);
            Z = zeros(4, 6);
            
            for i = 1:6
                X(:,i) = x(faces(i,:));
                Y(:,i) = y(faces(i,:));
                Z(:,i) = z(faces(i,:));
            end
        end

        %% Update graphics
        function updateGraphics(obj)
            if ~isfield(obj.graphics,'links') || isempty(obj.graphics.links)
                return;
            end
        
            % Update links
            for i = 1:obj.n
                if ~isfield(obj.graphics.links(i),'surf') || ...
                   ~isgraphics(obj.graphics.links(i).surf)
                    continue;
                end
        
                % Get transform of frame (i-1)
                T = obj.fk(i-1);
                
                % Origin of frame (i-1)
                p_frame = T(1:3,4);
                
                % Calculate link start and end based on type
                linkType = obj.visual.linkType{i};
                L = obj.visual.linkLengths(i);
                offset = obj.visual.linkOffset(i);
                
                if strcmpi(linkType, 'z')
                    % Case 1: Link extends along z-axis of frame (i-1)
                    z_axis = T(1:3,3);
                    p_start = p_frame + offset * z_axis;
                    p_end = p_start + L * z_axis;
                else  % 'xy'
                    % Case 2: Link in x-y plane, rotated by q(i) + offset from x-axis
                    x_axis = T(1:3,1);
                    y_axis = T(1:3,2);
                    z_axis = T(1:3,3);
                    p_start = p_frame + offset * z_axis;
                    theta = obj.q(i) + obj.visual.linkThetaOffset(i);
                    direction = cos(theta) * x_axis + sin(theta) * y_axis;
                    p_end = p_start + L * direction;
                end
                
                p_prev = p_start;
                p_curr = p_end;
        
                v = p_curr - p_prev;
                L = norm(v);
        
                if L < eps
                    continue;
                end
        
                [X,Y,Z] = cylinder(obj.visual.linkRadius(i), obj.visual.linkResolution);
                Z = Z * L;
        
                z0 = [0;0;1];
                R = vrrotvec2mat(vrrotvec(z0, v / L));
        
                P = R * [X(:)'; Y(:)'; Z(:)'];
                Xr = reshape(P(1,:), size(X)) + p_prev(1);
                Yr = reshape(P(2,:), size(Y)) + p_prev(2);
                Zr = reshape(P(3,:), size(Z)) + p_prev(3);
        
                set(obj.graphics.links(i).surf, ...
                    'XData',Xr,'YData',Yr,'ZData',Zr);
            end
            
            % Update frames
            if obj.visual.showFrames && isfield(obj.graphics,'frames') && ~isempty(obj.graphics.frames)
                obj.updateFrames();
            end
            
            % Update labels
            if obj.visual.showLabels && isfield(obj.graphics,'labels') && ~isempty(obj.graphics.labels)
                obj.updateLabels();
            end
            
            % Update end-effector
            if obj.visual.showEndEffector && isfield(obj.graphics,'endEffector')
                obj.updateEndEffector();
            end
        
            drawnow;
        end
        
        %% Update end-effector position
        function updateEndEffector(obj)
            if ~isfield(obj.graphics, 'endEffector') || isempty(obj.graphics.endEffector)
                return;
            end
            
            % Get current end-effector frame
            T_ee = obj.fk(obj.n);
            p = T_ee(1:3,4);
            x_axis = T_ee(1:3,1);
            y_axis = T_ee(1:3,2);
            z_axis = T_ee(1:3,3);
            R = [x_axis, y_axis, z_axis];
            
            size = obj.visual.endEffectorSize;
            
            switch obj.visual.endEffectorType
                case 'gripper'
                    % Update palm
                    if isfield(obj.graphics.endEffector, 'palm') && isgraphics(obj.graphics.endEffector.palm)
                        palmLength = size * 0.4;
                        palmWidth = size * 0.6;
                        palmThick = size * 0.15;
                        [X, Y, Z] = obj.createBox(palmLength, palmWidth, palmThick);
                        for i = 1:numel(X)
                            pt = R * [X(i); Y(i); Z(i)] + p;
                            X(i) = pt(1); Y(i) = pt(2); Z(i) = pt(3);
                        end
                        set(obj.graphics.endEffector.palm, 'XData', X, 'YData', Y, 'ZData', Z);
                    end
                    
                    % Update fingers
                    fingerLength = size * 0.8;
                    fingerWidth = size * 0.15;
                    fingerThick = size * 0.1;
                    fingerGap = size * 0.3;
                    
                    if isfield(obj.graphics.endEffector, 'finger1') && isgraphics(obj.graphics.endEffector.finger1)
                        [X1, Y1, Z1] = obj.createBox(fingerLength, fingerWidth, fingerThick);
                        offset1 = p + R * [palmLength/2; fingerGap/2 + fingerWidth/2; 0];
                        for i = 1:numel(X1)
                            pt = R * [X1(i); Y1(i); Z1(i)] + offset1;
                            X1(i) = pt(1); Y1(i) = pt(2); Z1(i) = pt(3);
                        end
                        set(obj.graphics.endEffector.finger1, 'XData', X1, 'YData', Y1, 'ZData', Z1);
                    end
                    
                    if isfield(obj.graphics.endEffector, 'finger2') && isgraphics(obj.graphics.endEffector.finger2)
                        [X2, Y2, Z2] = obj.createBox(fingerLength, fingerWidth, fingerThick);
                        offset2 = p + R * [palmLength/2; -fingerGap/2 - fingerWidth/2; 0];
                        for i = 1:numel(X2)
                            pt = R * [X2(i); Y2(i); Z2(i)] + offset2;
                            X2(i) = pt(1); Y2(i) = pt(2); Z2(i) = pt(3);
                        end
                        set(obj.graphics.endEffector.finger2, 'XData', X2, 'YData', Y2, 'ZData', Z2);
                    end
                    
                case 'tool'
                    if isfield(obj.graphics.endEffector, 'tool') && isgraphics(obj.graphics.endEffector.tool)
                        baseRadius = size * 0.3;
                        tipLength = size;
                        [X, Y, Z] = cylinder([baseRadius, 0], 20);
                        Z = Z * tipLength;
                        for i = 1:numel(X)
                            pt = R * [Z(i); X(i); Y(i)] + p;
                            X(i) = pt(1); Y(i) = pt(2); Z(i) = pt(3);
                        end
                        set(obj.graphics.endEffector.tool, 'XData', X, 'YData', Y, 'ZData', Z);
                    end
                    
                case 'sphere'
                    if isfield(obj.graphics.endEffector, 'sphere') && isgraphics(obj.graphics.endEffector.sphere)
                        [X, Y, Z] = sphere(20);
                        X = X * size * 0.5 + p(1);
                        Y = Y * size * 0.5 + p(2);
                        Z = Z * size * 0.5 + p(3);
                        set(obj.graphics.endEffector.sphere, 'XData', X, 'YData', Y, 'ZData', Z);
                    end
            end
        end
        
        %% Update frame graphics
        function updateFrames(obj)
            frameSize = obj.visual.frameSize;
            
            % Update base frame
            T = obj.baseT;
            obj.updateSingleFrame(obj.graphics.frames(1), T, frameSize);
            
            % Update all joint frames
            for i = 1:obj.n
                if length(obj.graphics.frames) < i+1
                    break;
                end
                T = obj.fk(i);
                obj.updateSingleFrame(obj.graphics.frames(i+1), T, frameSize);
            end
        end
        
        %% Update single frame
        function updateSingleFrame(obj, frameHandle, T, frameSize)
            if ~isfield(frameHandle,'x') || ~isgraphics(frameHandle.x)
                return;
            end
            
            p = T(1:3,4);
            x_axis = T(1:3,1);
            y_axis = T(1:3,2);
            z_axis = T(1:3,3);
            
            % Update X-axis
            set(frameHandle.x, 'XData', p(1), 'YData', p(2), 'ZData', p(3), ...
                'UData', x_axis(1)*frameSize, 'VData', x_axis(2)*frameSize, 'WData', x_axis(3)*frameSize);
            
            % Update Y-axis
            set(frameHandle.y, 'XData', p(1), 'YData', p(2), 'ZData', p(3), ...
                'UData', y_axis(1)*frameSize, 'VData', y_axis(2)*frameSize, 'WData', y_axis(3)*frameSize);
            
            % Update Z-axis
            set(frameHandle.z, 'XData', p(1), 'YData', p(2), 'ZData', p(3), ...
                'UData', z_axis(1)*frameSize, 'VData', z_axis(2)*frameSize, 'WData', z_axis(3)*frameSize);
        end
        
        %% Update label positions
        function updateLabels(obj)
            labelOffset = obj.visual.frameSize * 0.3;
            
            % Update base frame label
            T = obj.baseT;
            p = T(1:3,4);
            if isfield(obj.graphics.labels(1), 'text') && isgraphics(obj.graphics.labels(1).text)
                set(obj.graphics.labels(1).text, 'Position', [p(1)+labelOffset, p(2)+labelOffset, p(3)+labelOffset]);
            end
            
            % Update all frame labels
            for i = 1:obj.n
                if length(obj.graphics.labels) < i+1
                    break;
                end
                if ~isfield(obj.graphics.labels(i+1), 'text') || ~isgraphics(obj.graphics.labels(i+1).text)
                    continue;
                end
                
                T = obj.fk(i);
                p = T(1:3,4);
                offset = labelOffset * (1 + 0.3*mod(i,3));
                set(obj.graphics.labels(i+1).text, 'Position', [p(1)+offset, p(2)+offset, p(3)+offset]);
            end
        end
    end
end
