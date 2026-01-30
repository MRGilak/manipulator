classdef Simulation < handle
    properties
        robots          % Cell array of robot objects
        controllers     % Cell array of controller objects
        fig             % Figure handle
        ax              % Axes handle
        mode            % Simulation mode
        controls        % Structure to store UI control handles
        time            % Current simulation time
        dt              % Time step for dynamic simulation
    end
    
    methods
        function obj = Simulation(varargin)
            % Constructor: Simulation() or Simulation(robot1, robot2, ...)
            obj.robots = {};
            obj.mode = 'manual';
            obj.time = 0;
            obj.dt = 0.01;  % 10ms default time step
            obj.controls = struct();
            
            % Add any robots passed to constructor
            for i = 1:nargin
                obj.addRobot(varargin{i});
            end
        end
        
        function addRobot(obj, robot)
            % Add a robot to the simulation
            obj.robots{end+1} = robot;
        end

        function addController(obj, controller)
            % Add a robot to the simulation
            obj.controllers{end+1} = controller;
        end
        
        function removeRobot(obj, idx)
            % Remove a robot by index
            if idx >= 1 && idx <= length(obj.robots)
                obj.robots(idx) = [];
            end
        end
        
        function run(obj)
            % Create figure and UI, then run the simulation
            obj.createFigure();
            obj.createUI();
            obj.draw();
        end
        
        function createFigure(obj)
            % Create the main figure window
            obj.fig = figure('Name','Robot Simulation',...
                'NumberTitle','off',...
                'Position',[10 10 1000 800],...
                'CloseRequestFcn',@(~,~) obj.close());
            
            % Create 3D axes
            obj.ax = axes('Parent',obj.fig,...
                'Position',[0.05 0.05 0.65 0.9]);
            grid(obj.ax,'on');
            xlabel(obj.ax,'X (mm)');
            ylabel(obj.ax,'Y (mm)');
            zlabel(obj.ax,'Z (mm)');
            view(obj.ax,3);
            rotate3d(obj.ax,'on');
            
            hold(obj.ax,'on');
        end
        
        function createUI(obj)
            % Create UI controls based on current mode
            switch obj.mode
                case 'manual'
                    obj.createManualControls();
                case 'velocity'
                    obj.createVelocityControls();
                case 'trajectory'
                    obj.createTrajectoryControls();
                case 'dynamic'
                    obj.createDynamicControls();
            end
        end
        
        function createManualControls(obj)
            % Create sliders for manual joint control
            if isempty(obj.robots)
                return;
            end
            
            % For now, create controls for the first robot
            % Later can extend to multiple robots with tabs/selection
            robot = obj.robots{1};
            n = robot.n;
            
            % Mode selector
            uicontrol('Style','text',...
                'String','Mode:',...
                'Units','normalized',...
                'Position',[0.75 0.95 0.08 0.03],...
                'HorizontalAlignment','left');
            
            obj.controls.modeMenu = uicontrol('Style','popupmenu',...
                'String',{'Manual','Velocity','Trajectory','Dynamic'},...
                'Units','normalized',...
                'Position',[0.83 0.95 0.15 0.03],...
                'Callback',@(src,~) obj.changeMode(src.Value));
            
            % Joint sliders
            obj.controls.sliders = cell(n,1);
            obj.controls.sliderLabels = cell(n,1);
            obj.controls.valueLabels = cell(n,1);
            
            for i = 1:n
                % Joint name
                obj.controls.sliderLabels{i} = uicontrol('Style','text',...
                    'String',sprintf('Joint %d',i),...
                    'Units','normalized',...
                    'Position',[0.75 0.85-0.10*i 0.08 0.03],...
                    'HorizontalAlignment','left');
                
                % Slider
                obj.controls.sliders{i} = uicontrol('Style','slider',...
                    'Min',-pi,'Max',pi,'Value',robot.q(i),...
                    'Units','normalized',...
                    'Position',[0.75 0.82-0.10*i 0.18 0.04],...
                    'Callback',@(src,~) obj.sliderCallback(1,i,src.Value));
                
                % Value label
                obj.controls.valueLabels{i} = uicontrol('Style','text',...
                    'String',sprintf('%.2f°',rad2deg(robot.q(i))),...
                    'Units','normalized',...
                    'Position',[0.94 0.82-0.10*i 0.05 0.03],...
                    'HorizontalAlignment','left');
            end
            
            % Reset button
            obj.controls.resetBtn = uicontrol('Style','pushbutton',...
                'String','Reset Pose',...
                'Units','normalized',...
                'Position',[0.75 0.10 0.10 0.04],...
                'Callback',@(~,~) obj.resetPose());
            
            % Home button
            obj.controls.homeBtn = uicontrol('Style','pushbutton',...
                'String','Home Position',...
                'Units','normalized',...
                'Position',[0.86 0.10 0.12 0.04],...
                'Callback',@(~,~) obj.goHome());
        end
        
        function createVelocityControls(obj)
            % Create sliders for joint velocity control
            if isempty(obj.robots)
                return;
            end
            
            robot = obj.robots{1};
            n = robot.n;
            
            % Mode selector
            uicontrol('Style','text',...
                'String','Mode:',...
                'Units','normalized',...
                'Position',[0.75 0.95 0.08 0.03],...
                'HorizontalAlignment','left');
            
            obj.controls.modeMenu = uicontrol('Style','popupmenu',...
                'String',{'Manual','Velocity','Trajectory','Dynamic'},...
                'Value',2,...  % Velocity mode selected
                'Units','normalized',...
                'Position',[0.83 0.95 0.15 0.03],...
                'Callback',@(src,~) obj.changeMode(src.Value));
            
            % Velocity sliders
            obj.controls.sliders = cell(n,1);
            obj.controls.sliderLabels = cell(n,1);
            obj.controls.valueLabels = cell(n,1);
            
            maxVel = pi;  % Max velocity in rad/s
            
            for i = 1:n
                % Joint name
                obj.controls.sliderLabels{i} = uicontrol('Style','text',...
                    'String',sprintf('Joint %d',i),...
                    'Units','normalized',...
                    'Position',[0.75 0.85-0.10*i 0.08 0.03],...
                    'HorizontalAlignment','left');
                
                % Velocity slider
                obj.controls.sliders{i} = uicontrol('Style','slider',...
                    'Min',-maxVel,'Max',maxVel,'Value',0,...
                    'Units','normalized',...
                    'Position',[0.75 0.82-0.10*i 0.18 0.04],...
                    'Callback',@(src,~) obj.velocitySliderCallback(1,i,src.Value));
                
                % Value label
                obj.controls.valueLabels{i} = uicontrol('Style','text',...
                    'String','0.00 rad/s',...
                    'Units','normalized',...
                    'Position',[0.94 0.82-0.10*i 0.09 0.03],...
                    'HorizontalAlignment','left');
            end
            
            % Stop button
            obj.controls.stopBtn = uicontrol('Style','pushbutton',...
                'String','Stop All',...
                'Units','normalized',...
                'Position',[0.75 0.15 0.10 0.04],...
                'Callback',@(~,~) obj.stopAllVelocities());
            
            % Home button
            obj.controls.homeBtn = uicontrol('Style','pushbutton',...
                'String','Home Position',...
                'Units','normalized',...
                'Position',[0.86 0.15 0.12 0.04],...
                'Callback',@(~,~) obj.goHome());
            
            % Status text
            obj.controls.statusText = uicontrol('Style','text',...
                'String','Velocity Control Mode',...
                'Units','normalized',...
                'Position',[0.75 0.10 0.23 0.03],...
                'HorizontalAlignment','center',...
                'FontWeight','bold');
            
            % Create timer for continuous integration
            obj.controls.velocityTimer = timer(...
                'ExecutionMode','fixedRate',...
                'Period',0.01,...  
                'TimerFcn',@(~,~) obj.velocityTimerCallback());
            
            % Start the timer
            start(obj.controls.velocityTimer);
        end
        
        function createTrajectoryControls(obj)
            % Mode selector
            uicontrol('Style','text',...
                'String','Mode:',...
                'Units','normalized',...
                'Position',[0.75 0.95 0.08 0.03],...
                'HorizontalAlignment','left');
            
            obj.controls.modeMenu = uicontrol('Style','popupmenu',...
                'String',{'Manual','Velocity','Trajectory','Dynamic'},...
                'Value',3,...  % Trajectory mode selected
                'Units','normalized',...
                'Position',[0.83 0.95 0.15 0.03],...
                'Callback',@(src,~) obj.changeMode(src.Value));

            % Placeholder for trajectory mode controls
            uicontrol('Style','text',...
                'String','Trajectory Mode (Coming Soon)',...
                'Units','normalized',...
                'Position',[0.75 0.50 0.20 0.05],...
                'HorizontalAlignment','center');
        end
        
        function createDynamicControls(obj)
            % Dynamic simulation controls
            if isempty(obj.robots)
                return;
            end
                        
            % Mode selector
            uicontrol('Style','text',...
                'String','Mode:',...
                'Units','normalized',...
                'Position',[0.75 0.95 0.08 0.03],...
                'HorizontalAlignment','left');
            
            obj.controls.modeMenu = uicontrol('Style','popupmenu',...
                'String',{'Manual','Velocity','Trajectory','Dynamic'},...
                'Value',4,...
                'Units','normalized',...
                'Position',[0.83 0.95 0.15 0.03],...
                'Callback',@(src,~) obj.changeMode(src.Value));
            
            % Time display
            obj.controls.timeText = uicontrol('Style','text',...
                'String','Time: 0.00 s',...
                'Units','normalized',...
                'Position',[0.75 0.75 0.23 0.03],...
                'HorizontalAlignment','center',...
                'FontSize',10);
            
            % Start/Stop button
            obj.controls.startBtn = uicontrol('Style','pushbutton',...
                'String','Start Simulation',...
                'Units','normalized',...
                'Position',[0.75 0.65 0.23 0.06],...
                'FontSize',11,...
                'Callback',@(~,~) obj.toggleDynamicSim());
            
            % Reset button
            obj.controls.resetBtn = uicontrol('Style','pushbutton',...
                'String','Reset to Home',...
                'Units','normalized',...
                'Position',[0.75 0.55 0.23 0.05],...
                'Callback',@(~,~) obj.resetDynamics());
            
            % Status text
            obj.controls.statusText = uicontrol('Style','text',...
                'String','Ready',...
                'Units','normalized',...
                'Position',[0.75 0.45 0.23 0.05],...
                'HorizontalAlignment','center',...
                'FontWeight','bold',...
                'ForegroundColor',[0 0.5 0]);

            % Controller selector
            uicontrol('Style','text',...
                'String','Controller:',...
                'Units','normalized',...
                'Position',[0.75 0.35 0.08 0.03],...
                'HorizontalAlignment','left');
            
            obj.controls.controlMenu = uicontrol('Style','popupmenu',...
                'String',{'Open Loop','Gravity Compensation','PD', ...
                'PD With Gravity Compensation', 'Slotine'},...
                'Value',1,...
                'Units','normalized',...
                'Position',[0.83 0.35 0.15 0.03],...
                'Callback',@(src,~) obj.changeController(src.Value));

            % Plot Results
            obj.controls.plotResultsBtn = uicontrol('Style','pushbutton',...
                'String','Plot Results',...
                'Units','normalized',...
                'Position',[0.75 0.25 0.23 0.06],...
                'FontSize',11,...
                'Callback',@(~,~) obj.plotResults());
        end
        
        function draw(obj)
            % Draw all robots in the simulation
            cla(obj.ax);
            for i = 1:length(obj.robots)
                obj.robots{i}.draw(obj.ax);
            end
        end
        
        function sliderCallback(obj, robotIdx, jointIdx, value)
            % Handle slider value change
            if robotIdx > length(obj.robots)
                return;
            end
            
            robot = obj.robots{robotIdx};
            robot.q(jointIdx) = value;
            robot.updateGraphics();
            
            % Update value label
            if isfield(obj.controls, 'valueLabels') && ...
               length(obj.controls.valueLabels) >= jointIdx
                set(obj.controls.valueLabels{jointIdx}, ...
                    'String', sprintf('%.2f°', rad2deg(value)));
            end
        end
        
        function velocitySliderCallback(obj, robotIdx, jointIdx, value)
            % Handle velocity slider value change
            if robotIdx > length(obj.robots)
                return;
            end
            
            robot = obj.robots{robotIdx};
            robot.qdot(jointIdx) = value;
            
            % Update value label
            if isfield(obj.controls, 'valueLabels') && ...
               length(obj.controls.valueLabels) >= jointIdx
                set(obj.controls.valueLabels{jointIdx}, ...
                    'String', sprintf('%.2f rad/s', value));
            end
        end
        
        function velocityTimerCallback(obj)
            % Called periodically to integrate velocities
            if isempty(obj.robots)
                return;
            end
                        
            for i = 1:length(obj.robots)
                robot = obj.robots{i};
                
                % Integrate joint velocities
                robot.integrateJointVelocities(obj.dt);
                
                % Update graphics
                robot.updateGraphics();
            end
        end
        
        function stopAllVelocities(obj)
            % Set all joint velocities to zero
            for i = 1:length(obj.robots)
                obj.robots{i}.qdot = zeros(obj.robots{i}.n, 1);
            end
            
            % Reset sliders to zero
            if isfield(obj.controls, 'sliders')
                for j = 1:length(obj.controls.sliders)
                    set(obj.controls.sliders{j}, 'Value', 0);
                    set(obj.controls.valueLabels{j}, 'String', '0.00 rad/s');
                end
            end
        end
        
        function changeMode(obj, modeIdx)
            % Change simulation mode
            modes = {'manual', 'velocity', 'trajectory', 'dynamic'};
            obj.mode = modes{modeIdx};
            
            % Clear existing controls
            obj.clearControls();
            
            % Create new controls for selected mode
            obj.createUI();
        end

        function changeController(obj, controllerIdx)
            % Change simulation mode
            types = {'Open-Loop', 'Gravity Compensation', 'PD', 'PD With Gravity Compensation', 'Slotine'};
            obj.controllers{1}.type = types{controllerIdx};
        end
        
        function clearControls(obj)
            % Stop velocity timer if running
            if isfield(obj.controls, 'velocityTimer') && ...
               isvalid(obj.controls.velocityTimer)
                stop(obj.controls.velocityTimer);
                delete(obj.controls.velocityTimer);
            end
            
            % Remove all UI controls
            fields = fieldnames(obj.controls);
            for i = 1:length(fields)
                field = fields{i};
                if iscell(obj.controls.(field))
                    for j = 1:length(obj.controls.(field))
                        if ishandle(obj.controls.(field){j})
                            delete(obj.controls.(field){j});
                        end
                    end
                elseif ishandle(obj.controls.(field))
                    delete(obj.controls.(field));
                end
            end
            obj.controls = struct();
        end
        
        function resetPose(obj)
            % Reset all robots to current pose 
            
        end
        
        function goHome(obj)
            % Move all robots to home position (q = 0)
            for i = 1:length(obj.robots)
                obj.robots{i}.q = zeros(obj.robots{i}.n, 1);
                obj.robots{i}.updateGraphics();
            end
            
            % Update slider positions
            if isfield(obj.controls, 'sliders')
                for j = 1:length(obj.controls.sliders)
                    set(obj.controls.sliders{j}, 'Value', 0);
                    set(obj.controls.valueLabels{j}, 'String', '0.00°');
                end
            end
        end
        
        function startDynamicSim(obj)
            % Start dynamic simulation with timer
            if isempty(obj.robots)
                return;
            end
            
            % Create and start timer for dynamics integration
            obj.controls.dynamicsTimer = timer(...
                'ExecutionMode','fixedRate',...
                'Period',obj.dt,...
                'TimerFcn',@(~,~) obj.dynamicsTimerCallback());
            
            obj.time = 0;
            start(obj.controls.dynamicsTimer);
            
            % Update UI
            set(obj.controls.startBtn, 'String', 'Stop Simulation');
            set(obj.controls.statusText, 'String', 'Running', 'ForegroundColor', [1 0 0]);
        end
        
        function stopDynamicSim(obj)
            % Stop dynamic simulation
            if isfield(obj.controls, 'dynamicsTimer') && ...
               isvalid(obj.controls.dynamicsTimer)
                stop(obj.controls.dynamicsTimer);
                delete(obj.controls.dynamicsTimer);
            end
            
            % Update UI
            set(obj.controls.startBtn, 'String', 'Start Simulation');
            set(obj.controls.statusText, 'String', 'Stopped', 'ForegroundColor', [1 0.5 0]);
        end
        
        function toggleDynamicSim(obj)
            % Toggle simulation on/off
            if isfield(obj.controls, 'dynamicsTimer') && ...
               isvalid(obj.controls.dynamicsTimer) && ...
               strcmp(obj.controls.dynamicsTimer.Running, 'on')
                obj.stopDynamicSim();
            else
                obj.startDynamicSim();
            end
        end
        
        function dynamicsTimerCallback(obj)
            % Dynamic simulation integration: D*q_ddot + C*q_dot + G = u
            if isempty(obj.robots)
                return;
            end
            
            robot = obj.robots{1};
            
            % Compute dynamics matrices
            D = robot.inertiaMatrix();
            C = robot.coriolisCentrifugalMatrix(obj.dt);
            G = robot.gravityTorque();
            
            controller = obj.controllers{1};
            u = controller.uNext();

            % G = zeros(6, 1);
            % u = 1e7 * [0; 0; 0; 0; sin(2*pi*2*obj.time); 0];

            % Solve for acceleration
            q_ddot = D \ (-C * robot.qdot - G + u);
             
            % Integrate velocity
            robot.qdot = robot.qdot + q_ddot * obj.dt;
            
            % Integrate position
            robot.q = robot.q + robot.qdot * obj.dt;

            % Save results
            robot.q_his(:, end+1) = robot.q;
            robot.qdot_his(:, end+1) = robot.qdot;
            robot.qddot_his(:, end+1) = q_ddot;
            robot.u_his(:, end+1) = u;
            
            % Update visualization
            robot.updateGraphics();
            
            % Update time and display
            obj.time = obj.time + obj.dt;
            set(obj.controls.timeText, 'String', sprintf('Time: %.2f s', obj.time));
            drawnow limitrate;
        end

        function resetDynamics(obj)
            % Reset to home position and zero velocities
            if isempty(obj.robots)
                return;
            end
            
            % Stop simulation if running
            if isfield(obj.controls, 'dynamicsTimer') && ...
               isvalid(obj.controls.dynamicsTimer)
                obj.stopDynamicSim();
            end
            
            % Reset robot state
            for i = 1:length(obj.robots)
                obj.robots{i}.q = zeros(obj.robots{i}.n, 1);
                obj.robots{i}.qdot = zeros(obj.robots{i}.n, 1);
                obj.robots{i}.updateGraphics();
            end
            
            % Reset time
            obj.time = 0;
            set(obj.controls.timeText, 'String', 'Time: 0.00 s');
            set(obj.controls.statusText, 'String', 'Ready', 'ForegroundColor', [0 0.5 0]);
        end
        
        function close(obj)
            % Stop and delete velocity timer if it exists
            if isfield(obj.controls, 'velocityTimer') && ...
               isvalid(obj.controls.velocityTimer)
                stop(obj.controls.velocityTimer);
                delete(obj.controls.velocityTimer);
            end
            
            % Stop and delete dynamics timer if it exists
            if isfield(obj.controls, 'dynamicsTimer') && ...
               isvalid(obj.controls.dynamicsTimer)
                stop(obj.controls.dynamicsTimer);
                delete(obj.controls.dynamicsTimer);
            end
            
            % Clean up and close simulation
            if ishandle(obj.fig)
                delete(obj.fig);
            end
        end

        function plotResults(obj)
            robot = obj.robots{1};
            tf = length(robot.q_his)*obj.dt;
            t = obj.dt:obj.dt:tf;

            % Angles
            figure("Name", "Joint Angles", 'Position', [10, 10, 1000, 800]);
            subplot(2, 3, 1);
            plot(t, robot.q_his(1, :), 'b', 'LineWidth', 1.5);
            grid on;
            title('$q_1 (t)$', 'Interpreter', 'latex', 'FontName', ...
                'Times New Roman', 'FontSize', 9);
            xlabel('Time (s)', 'FontName', 'Times New Roman', ...
                'FontSize', 9);
            ylabel('Angle ($^\circ$)', 'interpreter', 'latex', ...
                'FontName', 'Times New Roman', 'FontSize', 9);

            subplot(2, 3, 2);
            plot(t, robot.q_his(2, :), 'b', 'LineWidth', 1.5);
            grid on;
            title('$q_2 (t)$', 'Interpreter', 'latex', 'FontName', ...
                'Times New Roman', 'FontSize', 9);
            xlabel('Time (s)', 'FontName', 'Times New Roman', ...
                'FontSize', 9);
            ylabel('Angle ($^\circ$)', 'interpreter', 'latex', ...
                'FontName', 'Times New Roman', 'FontSize', 9);

            subplot(2, 3, 3);
            plot(t, robot.q_his(3, :), 'b', 'LineWidth', 1.5);
            grid on;
            title('$q_3 (t)$', 'Interpreter', 'latex', 'FontName', ...
                'Times New Roman', 'FontSize', 9);
            xlabel('Time (s)', 'FontName', 'Times New Roman', ...
                'FontSize', 9);
            ylabel('Angle ($^\circ$)', 'interpreter', 'latex', ...
                'FontName', 'Times New Roman', 'FontSize', 9);

            subplot(2, 3, 4);
            plot(t, robot.q_his(4, :), 'b', 'LineWidth', 1.5);
            grid on;
            title('$q_4 (t)$', 'Interpreter', 'latex', 'FontName', ...
                'Times New Roman', 'FontSize', 9);
            xlabel('Time (s)', 'FontName', 'Times New Roman', ...
                'FontSize', 9);
            ylabel('Angle ($^\circ$)', 'interpreter', 'latex', ...
                'FontName', 'Times New Roman', 'FontSize', 9);

            subplot(2, 3, 5);
            plot(t, robot.q_his(5, :), 'b', 'LineWidth', 1.5);
            grid on;
            title('$q_5 (t)$', 'Interpreter', 'latex', 'FontName', ...
                'Times New Roman', 'FontSize', 9);
            xlabel('Time (s)', 'FontName', 'Times New Roman', ...
                'FontSize', 9);
            ylabel('Angle ($^\circ$)', 'interpreter', 'latex', ...
                'FontName', 'Times New Roman', 'FontSize', 9);

            subplot(2, 3, 6);
            plot(t, robot.q_his(6, :), 'b', 'LineWidth', 1.5);
            grid on;
            title('$q_6 (t)$', 'Interpreter', 'latex', 'FontName', ...
                'Times New Roman', 'FontSize', 9);
            xlabel('Time (s)', 'FontName', 'Times New Roman', ...
                'FontSize', 9);
            ylabel('Angle ($^\circ$)', 'interpreter', 'latex', ...
                'FontName', 'Times New Roman', 'FontSize', 9);
            
        end
    end
end
