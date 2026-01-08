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

        % Base pose
        baseT

        % Visualization
        visual
        graphics
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

            % Base pose
            if nargin > 4
                obj.baseT = varargin{1};
            else
                obj.baseT = eye(4);
            end

            % Visual defaults
            obj.visual = struct();
            obj.visual.linkRadius     = 20;
            obj.visual.linkResolution = 20;
            
            % Link lengths (physical length of each link)
            obj.visual.linkLengths = sqrt(obj.a.^2 + obj.d.^2);  % Default estimate
            
            % Link offset: distance from frame (i-1) origin where link starts
            obj.visual.linkOffset = zeros(1, obj.n);
            
            % Theta offset: additional rotation in x-y plane (for 'xy' type links)
            % Final angle = q(i) + linkThetaOffset(i)
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

            obj.graphics = struct();
        end

        %% Set joint angles
        function setJointAngles(obj,q)
            obj.q = q(:);
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

            Ti = dh(theta,d_i,a_i,alpha_i);
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

        %% Draw robot
        function draw(obj, ax)
            hold(ax,'on');
            axis(ax,'equal');
            axis(ax,'vis3d');
            grid(ax,'on');
            view(ax,3);
        
            xlabel(ax,'X');
            ylabel(ax,'Y');
            zlabel(ax,'Z');
        
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
                [X,Y,Z] = cylinder(obj.visual.linkRadius, obj.visual.linkResolution);
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
        
                [X,Y,Z] = cylinder(obj.visual.linkRadius, obj.visual.linkResolution);
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
        
            drawnow;
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
