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
        plotConfig      % Configuration for plotting
        controlTime     % Time since last control update
        lastControl     % Last computed control input (zero-order hold)
        headless        % Boolean flag for headless mode (no graphics)
        
        % Task-space trajectory tracking
        trajectoryFunc  % Function handle: @(t) returns [R, O]
        useTrajectory   % Boolean flag for trajectory tracking
        R_prev          % Previous orientation for numerical differentiation
        O_prev          % Previous position for numerical differentiation
        
        % History tracking for desired values
        qdes_his        % Desired joint positions history
        qdotdes_his     % Desired joint velocities history
        O_his           % End-effector position history
        Odes_his        % Desired end-effector position history
        F_des_his       % Desired constraint force history
    end
    
    methods
        function obj = Simulation(varargin)
            % Constructor: Simulation() or Simulation(robot1, robot2, ...)
            obj.robots = {};
            obj.mode = 'manual';
            obj.time = 0;
            obj.dt = 0.01;
            obj.controls = struct();
            obj.headless = false;
            
            obj.plotConfig = struct('q', true, 'qdot', false, 'qddot', false, 'u', false, ...
                'saveResults', false, 'saveFilename', 'sim_results.mat');
            obj.controlTime = 0;
            obj.lastControl = [];
            
            % Trajectory tracking
            obj.trajectoryFunc = [];
            obj.useTrajectory = false;
            obj.R_prev = [];
            obj.O_prev = [];
            
            % History tracking
            obj.qdes_his = [];
            obj.qdotdes_his = [];
            obj.O_his = [];
            obj.Odes_his = [];
            obj.F_des_his = [];
            
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
        
        function run(obj, varargin)
            % Create figure and UI, then run the simulation            
            % Parse optional arguments
            p = inputParser;
            addParameter(p, 'draw', true, @islogical);
            addParameter(p, 'headless', false, @islogical);
            addParameter(p, 'duration', 0, @isnumeric);  % Auto-run duration (0 = manual mode)
            parse(p, varargin{:});
            
            % Store settings
            obj.headless = p.Results.headless;
            obj.plotConfig.drawRobot = p.Results.draw && ~obj.headless;
            
            % Only create figure and UI if not in headless mode
            if ~obj.headless
                obj.createFigure();
                obj.createUI();
                
                % Only draw if enabled
                if obj.plotConfig.drawRobot
                    obj.draw();
                end
            else
                % In headless mode with auto-run duration, start simulation automatically
                if p.Results.duration > 0 && ismember(obj.mode, {'dynamic', 'task-space'})
                    obj.runHeadless(p.Results.duration);
                end
            end
        end
        
        function runHeadless(obj, duration)
            % Run simulation in headless mode for a specified duration
            if isempty(obj.robots)
                return;
            end
            
            fprintf('Running headless simulation for %.2f seconds...\n', duration);
            
            % Start the simulation
            obj.startDynamicSim();
            
            % Wait for the specified duration
            tic;
            while toc < duration
                pause(0.1);  % Small pause to prevent CPU overload
            end
            
            % Stop the simulation
            obj.stopDynamicSim();
            
            fprintf('Simulation complete. Time: %.2f s\n', obj.time);
            
            % Auto-save results if enabled
            if obj.plotConfig.saveResults
                obj.saveResults();
            end
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
                case 'task-space'
                    obj.createTaskSpaceControls();
                case 'constrained'
                    obj.createConstrainedControls();
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
                'String',{'Manual','Velocity','Trajectory','Dynamic','Task-Space'},...
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
                'String',{'Manual','Velocity','Trajectory','Dynamic','Task-Space','Constrained'},...
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
        
        function createTaskSpaceControls(obj)
            % Task-space simulation controls
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
                'String',{'Manual','Velocity','Trajectory','Dynamic','Task-Space','Constrained'},...
                'Value',5,...
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
                'Callback',@(~,~) obj.toggleTaskSpaceSim());
            
            % Reset button
            obj.controls.resetBtn = uicontrol('Style','pushbutton',...
                'String','Reset to Home',...
                'Units','normalized',...
                'Position',[0.75 0.55 0.23 0.05],...
                'Callback',@(~,~) obj.resetDynamics());
            
            % Status text
            obj.controls.statusText = uicontrol('Style','text',...
                'String','Ready - Task-Space Control',...
                'Units','normalized',...
                'Position',[0.75 0.45 0.23 0.05],...
                'HorizontalAlignment','center',...
                'FontWeight','bold',...
                'ForegroundColor',[0 0.5 0]);

            % Plot Results
            obj.controls.plotResultsBtn = uicontrol('Style','pushbutton',...
                'String','Plot Results',...
                'Units','normalized',...
                'Position',[0.75 0.25 0.23 0.06],...
                'FontSize',11,...
                'Callback',@(~,~) obj.plotResults());
        end
        
        function createConstrainedControls(obj)
            % Constrained dynamic simulation controls
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
                'String',{'Manual','Velocity','Trajectory','Dynamic','Task-Space','Constrained'},...
                'Value',6,...
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
                'String','Ready - Constrained Dynamics',...
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
            modes = {'manual', 'velocity', 'trajectory', 'dynamic', 'task-space', 'constrained'};
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
            
            % Update UI first (only if not headless)
            if ~obj.headless
                set(obj.controls.startBtn, 'String', 'Stop Simulation');
                set(obj.controls.statusText, 'String', 'Running', 'ForegroundColor', [1 0 0]);
            end
            
            % Create and start timer for dynamics integration
            obj.controls.dynamicsTimer = timer(...
                'ExecutionMode','fixedRate',...
                'Period',obj.dt,...
                'TimerFcn',@(~,~) obj.dynamicsTimerCallback());
            
            obj.time = 0;
            obj.controlTime = 0;
            obj.lastControl = [];
            
            % Reset history tracking
            obj.qdes_his = [];
            obj.qdotdes_his = [];
            obj.O_his = [];
            obj.Odes_his = [];
            obj.F_des_his = [];
            
            % Reset robot history
            robot = obj.robots{1};
            robot.q_his = [];
            robot.qdot_his = [];
            robot.qddot_his = [];
            robot.u_his = [];
            
            start(obj.controls.dynamicsTimer);
        end
        
        function stopDynamicSim(obj)
            % Stop dynamic simulation
            if isfield(obj.controls, 'dynamicsTimer') && ...
               isvalid(obj.controls.dynamicsTimer)
                stop(obj.controls.dynamicsTimer);
                delete(obj.controls.dynamicsTimer);
            end
            
            % Auto-save results if enabled
            if obj.plotConfig.saveResults
                obj.saveResults();
            end
            
            % Update UI (only if not headless)
            if ~obj.headless
                set(obj.controls.startBtn, 'String', 'Start Simulation');
                set(obj.controls.statusText, 'String', 'Stopped', 'ForegroundColor', [1 0.5 0]);
            end
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
        
        function toggleTaskSpaceSim(obj)
            % Toggle task-space simulation on/off
            if isfield(obj.controls, 'dynamicsTimer') && ...
               isvalid(obj.controls.dynamicsTimer) && ...
               strcmp(obj.controls.dynamicsTimer.Running, 'on')
                obj.stopDynamicSim();
            else
                obj.startDynamicSim();
            end
        end
        
        function dynamicsTimerCallback(obj)
            if isempty(obj.robots)
                return;
            end
            
            robot = obj.robots{1};
            controller = obj.controllers{1};
            
            % Update desired trajectory if using task-space trajectory
            if obj.useTrajectory && ~isempty(obj.trajectoryFunc)
                obj.updateDesiredFromTrajectory();
            end
            
            % Zero-order hold: compute control at controller rate
            if obj.controlTime >= controller.dt || isempty(obj.lastControl)
                obj.lastControl = controller.uNext();
                obj.controlTime = 0;
            end
            u = obj.lastControl;
            
            % Compute dynamics based on mode
            if strcmp(obj.mode, 'constrained')
                % Use constrained dynamics
                [q_ddot, lambda, F] = robot.constrainedDynamics(u, obj.dt);
                
                % Save constraint forces
                if isempty(robot.lambda_his)
                    robot.lambda_his = lambda;
                    robot.F_his = F;
                else
                    robot.lambda_his(:, end+1) = lambda;
                    robot.F_his(:, end+1) = F;
                end
                
                % Save desired constraint force if using Force-motion controller
                if strcmp(controller.type, 'Force-motion') && ~isempty(controller.F_des)
                    if isempty(obj.F_des_his)
                        obj.F_des_his = controller.F_des;
                    else
                        obj.F_des_his(:, end+1) = controller.F_des;
                    end
                end
            else
                % Standard unconstrained dynamics
                D = robot.inertiaMatrix();
                C = robot.coriolisCentrifugalMatrix(obj.dt);
                G = robot.gravityTorque();

                % Solve for acceleration
                q_ddot = D \ (-C * robot.qdot - G + u);
            end
             
            % Integrate velocity
            robot.qdot = robot.qdot + q_ddot * obj.dt;
            
            % Integrate position
            robot.q = robot.q + robot.qdot * obj.dt;

            % Optional debug checks
            if isfield(obj.plotConfig, 'debugInertia') && obj.plotConfig.debugInertia
                robot.checkInertiaMatrixPD();
            end
            if isfield(obj.plotConfig, 'debugSkewSymmetry') && obj.plotConfig.debugSkewSymmetry
                robot.checkSkewSymmetry(obj.dt);
            end

            % Save results
            robot.q_his(:, end+1) = robot.q;
            robot.qdot_his(:, end+1) = robot.qdot;
            robot.qddot_his(:, end+1) = q_ddot;
            robot.u_his(:, end+1) = u;
            
            % Save desired values if available
            if ~isempty(controller.qdes)
                obj.qdes_his(:, end+1) = controller.qdes;
            end
            if ~isempty(controller.qdotdes)
                obj.qdotdes_his(:, end+1) = controller.qdotdes;
            end
            
            % Save end-effector position for task-space tracking
            if obj.useTrajectory
                T_current = robot.fk(robot.n);
                obj.O_his(:, end+1) = T_current(1:3, 4);
                if ~isempty(obj.trajectoryFunc) && ~isempty(controller.qdes)
                    % Store the FK of controller.qdes as the actual desired trajectory
                    q_save = robot.q;  % Save current state
                    robot.q = controller.qdes;  % Temporarily set to desired
                    T_des = robot.fk(robot.n);
                    robot.q = q_save;  % Restore current state
                    obj.Odes_his(:, end+1) = T_des(1:3, 4);
                end
            end

            % Update visualization (only if not headless)
            if ~obj.headless
                robot.updateGraphics();
            end
            
            % Update time and display
            obj.time = obj.time + obj.dt;
            obj.controlTime = obj.controlTime + obj.dt;
            
            % Update UI (only if not headless)
            if ~obj.headless
                set(obj.controls.timeText, 'String', sprintf('Time: %.2f s', obj.time));
                drawnow limitrate;
            end
        end

        function updateDesiredFromTrajectory(obj)
            % Compute qdes and qdotdes from task-space trajectory
            robot = obj.robots{1};
            controller = obj.controllers{1};
            
            % Get desired pose from trajectory
            % Check if trajectory function returns velocities
            nout = nargout(obj.trajectoryFunc);
            if nout >= 4
                % Trajectory provides velocities
                [R_d, O_d, v_d_linear, omega_d] = obj.trajectoryFunc(obj.time);
                v_d = [v_d_linear; omega_d];
            else
                % Trajectory only provides pose, compute velocities numerically
                [R_d, O_d] = obj.trajectoryFunc(obj.time);
                
                % Numerical differentiation for velocity
                if isempty(obj.R_prev) || isempty(obj.O_prev)
                    % First iteration: initialize
                    O_d_dot = zeros(3, 1);
                    omega_d = zeros(3, 1);
                else
                    % Compute O_dot numerically
                    O_d_dot = (O_d - obj.O_prev) / obj.dt;
                    
                    % Compute omega from R_dot
                    R_d_dot = (R_d - obj.R_prev) / obj.dt;
                    S_omega = R_d_dot * R_d';
                    omega_d = robot.unskew(S_omega);
                end
                
                v_d = [O_d_dot; omega_d];
            end
            
            % Store for next iteration
            obj.R_prev = R_d;
            obj.O_prev = O_d;
            
            % Compute desired joint velocity using damped least squares
            J = robot.jacobian(robot.n);
            lambda_dls = 0.01;
            qdot_des = J' * ((J * J' + lambda_dls^2 * eye(6)) \ v_d);
            
            % Integrate to get qdes
            if isempty(controller.qdes)
                % Initialize using IK to match the desired trajectory at t=0
                T_d = [R_d, O_d; 0 0 0 1];
                controller.qdes = robot.ik(T_d, robot.q);
            end
            controller.qdes = controller.qdes + qdot_des * obj.dt;
            
            % For Force-motion controller, compute qddotdes and F_des first (before updating qdotdes)
            if strcmp(controller.type, 'Force-motion')
                % Compute qddotdes from velocity derivative (use old qdotdes)
                if isempty(controller.qdotdes) || length(controller.qdotdes) ~= robot.n
                    qdot_des_prev = zeros(robot.n, 1);
                elseif all(controller.qdotdes == 0)
                    qdot_des_prev = zeros(robot.n, 1);
                else
                    qdot_des_prev = controller.qdotdes;
                end
                qddot_des = (qdot_des - qdot_des_prev) / obj.dt;
                controller.qddotdes = qddot_des;
                
                % Compute desired constraint force to maintain constraint
                if ~isempty(robot.J_e)
                    % Map gravity torques to task space using pseudo-inverse of J'
                    G = robot.gravityTorque();
                    J_pinv_T = pinv(J');  % Pseudo-inverse of J transpose
                    F_gravity = J_pinv_T * G;  % Task-space gravity equivalent
                    
                    % For z-constraint, desired force is primarily in z-direction
                    % Use projection of gravity force onto constraint direction
                    controller.F_des = [0; 0; F_gravity(3); 0; 0; 0];
                end
            end
            
            % Update qdotdes after computing qddotdes
            controller.qdotdes = qdot_des;
            
            % For Slotine controller, also compute qddotdes
            if strcmp(controller.type, 'Slotine')
                if isempty(controller.qddotdes)
                    controller.qddotdes = zeros(robot.n, 1);
                end
            end
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
                obj.robots{i}.q_his = [];
                obj.robots{i}.qdot_his = [];
                obj.robots{i}.qddot_his = [];
                obj.robots{i}.u_his = [];
                obj.robots{i}.lambda_his = [];
                obj.robots{i}.F_his = [];
                if ~obj.headless
                    obj.robots{i}.updateGraphics();
                end
            end
            
            % Reset trajectory tracking
            obj.R_prev = [];
            obj.O_prev = [];
            
            % Reset history tracking
            obj.qdes_his = [];
            obj.qdotdes_his = [];
            obj.O_his = [];
            obj.Odes_his = [];
            obj.F_des_his = [];
            
            % Reset time
            obj.time = 0;
            obj.controlTime = 0;
            obj.lastControl = [];
            
            % Update UI (only if not headless)
            if ~obj.headless
                set(obj.controls.timeText, 'String', 'Time: 0.00 s');
                set(obj.controls.statusText, 'String', 'Ready', 'ForegroundColor', [0 0.5 0]);
            end
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

            if obj.plotConfig.q
                if ~isempty(obj.qdes_his)
                    obj.plotVariableWithRef(t, robot.q_his, obj.qdes_his, 'q', 'Angle (rad)');
                else
                    obj.plotVariable(t, robot.q_his, 'q', 'Angle (rad)');
                end
            end
            if obj.plotConfig.qdot
                if ~isempty(obj.qdotdes_his)
                    obj.plotVariableWithRef(t, robot.qdot_his, obj.qdotdes_his, 'qdot', 'Velocity (rad/s)');
                else
                    obj.plotVariable(t, robot.qdot_his, 'qdot', 'Velocity (rad/s)');
                end
            end
            if obj.plotConfig.qddot
                obj.plotVariable(t, robot.qddot_his, 'qddot', 'Acceleration (rad/s^2)');
            end
            if obj.plotConfig.u
                obj.plotVariable(t, robot.u_his, 'u', 'Torque (N.mm)');
            end
            
            % Plot constraint forces if in constrained mode
            if strcmp(obj.mode, 'constrained') && ~isempty(robot.lambda_his)
                obj.plotVariable(t, robot.lambda_his, 'lambda', 'Lagrange Multipliers');
                if ~isempty(obj.F_des_his)
                    obj.plotVariableWithRef(t, robot.F_his, obj.F_des_his, 'F', 'Constraint Forces (N)');
                else
                    obj.plotVariable(t, robot.F_his, 'F', 'Constraint Forces (N)');
                end
            end
            
            % Plot end-effector position for task-space mode
            if obj.useTrajectory && ~isempty(obj.O_his)
                obj.plotEndEffector(t);
                obj.plotCircularTrajectory2D(t);
            end
        end
        
        function plotVariable(obj, t, data, varName, yLabel)
            n = size(data, 1);
            figure('Name', sprintf('Joint %s', varName), 'Position', [10, 10, 1000, 800]);
            for i = 1:n
                subplot(2, 3, i);
                plot(t, data(i, :), 'b', 'LineWidth', 1.5);
                grid on;
                xlim([0 max(t)]);
                title(sprintf('$%s_%d (t)$', varName, i), 'Interpreter', 'latex', ...
                    'FontName', 'Times New Roman', 'FontSize', 9);
                xlabel('Time (s)', 'FontName', 'Times New Roman', 'FontSize', 9);
                ylabel(yLabel, 'interpreter', 'latex', ...
                    'FontName', 'Times New Roman', 'FontSize', 9);
            end
        end
        
        function plotVariableWithRef(obj, t, data, dataRef, varName, yLabel)
            n = size(data, 1);
            
            % Ensure dataRef matches the size of data
            minLen = min(size(data, 2), size(dataRef, 2));
            if size(data, 2) ~= size(dataRef, 2)
                warning('Data and reference have different lengths (%d vs %d). Truncating to %d samples.', ...
                    size(data, 2), size(dataRef, 2), minLen);
                data = data(:, 1:minLen);
                dataRef = dataRef(:, 1:minLen);
                t = t(1:minLen);
            end
            
            figure('Name', sprintf('Joint %s', varName), 'Position', [10, 10, 1000, 800]);
            for i = 1:n
                subplot(2, 3, i);
                plot(t, data(i, :), 'b', 'LineWidth', 1.5); hold on;
                plot(t, dataRef(i, :), 'r--', 'LineWidth', 1.5);
                grid on;
                xlim([0 max(t)]);
                legend('Actual', 'Desired', 'Location', 'best');
                title(sprintf('$%s_%d (t)$', varName, i), 'Interpreter', 'latex', ...
                    'FontName', 'Times New Roman', 'FontSize', 9);
                xlabel('Time (s)', 'FontName', 'Times New Roman', 'FontSize', 9);
                ylabel(yLabel, 'interpreter', 'latex', ...
                    'FontName', 'Times New Roman', 'FontSize', 9);
            end
        end
        
        function plotEndEffector(obj, t)
            % Plot end-effector position in task-space
            figure('Name', 'End-Effector Position', 'Position', [10 10 1000 800]);
            
            % 3D trajectory plot
            subplot(2, 2, 1);
            plot3(obj.O_his(1, :), obj.O_his(2, :), obj.O_his(3, :), 'b', 'LineWidth', 1.5); hold on;
            if ~isempty(obj.Odes_his)
                plot3(obj.Odes_his(1, :), obj.Odes_his(2, :), obj.Odes_his(3, :), 'r--', 'LineWidth', 1.5);
                legend('Actual', 'Desired', 'Location', 'best');
            end
            grid on;
            xlabel('X (mm)', 'FontName', 'Times New Roman');
            ylabel('Y (mm)', 'FontName', 'Times New Roman');
            zlabel('Z (mm)', 'FontName', 'Times New Roman');
            title('End-Effector 3D Trajectory', 'FontName', 'Times New Roman');
            view(3);
            axis equal;
            
            % X vs time
            subplot(2, 2, 2);
            plot(t, obj.O_his(1, :), 'b', 'LineWidth', 1.5); hold on;
            if ~isempty(obj.Odes_his)
                plot(t, obj.Odes_his(1, :), 'r--', 'LineWidth', 1.5);
                legend('Actual', 'Desired', 'Location', 'best');
            end
            grid on;
            xlim([0 max(t)]);
            xlabel('Time (s)', 'FontName', 'Times New Roman');
            ylabel('X (mm)', 'FontName', 'Times New Roman');
            title('End-Effector X Position', 'FontName', 'Times New Roman');
            
            % Y vs time
            subplot(2, 2, 3);
            plot(t, obj.O_his(2, :), 'b', 'LineWidth', 1.5); hold on;
            if ~isempty(obj.Odes_his)
                plot(t, obj.Odes_his(2, :), 'r--', 'LineWidth', 1.5);
                legend('Actual', 'Desired', 'Location', 'best');
            end
            grid on;
            xlim([0 max(t)]);
            xlabel('Time (s)', 'FontName', 'Times New Roman');
            ylabel('Y (mm)', 'FontName', 'Times New Roman');
            title('End-Effector Y Position', 'FontName', 'Times New Roman');
            
            % Z vs time
            subplot(2, 2, 4);
            plot(t, obj.O_his(3, :), 'b', 'LineWidth', 1.5); hold on;
            if ~isempty(obj.Odes_his)
                plot(t, obj.Odes_his(3, :), 'r--', 'LineWidth', 1.5);
                legend('Actual', 'Desired', 'Location', 'best');
            end
            grid on;
            xlim([0 max(t)]);
            xlabel('Time (s)', 'FontName', 'Times New Roman');
            ylabel('Z (mm)', 'FontName', 'Times New Roman');
            title('End-Effector Z Position', 'FontName', 'Times New Roman');
        end
        
        function plotCircularTrajectory2D(obj, t)
            % Plot 2D circular trajectory with time-based coloring
            figure('Name', 'Circular Trajectory 2D', 'Position', [10 10 1000 800]);
            
            % 2D trajectory with time coloring (actual)
            subplot(1, 2, 1);
            scatter(obj.O_his(1, :), obj.O_his(2, :), 30, t, 'filled');
            hold on;
            if ~isempty(obj.Odes_his)
                plot(obj.Odes_his(1, :), obj.Odes_his(2, :), 'r--', 'LineWidth', 1.5);
            end
            colorbar;
            colormap('jet');
            grid on;
            axis equal;
            xlabel('X (mm)', 'FontName', 'Times New Roman', 'FontSize', 11);
            ylabel('Y (mm)', 'FontName', 'Times New Roman', 'FontSize', 11);
            title('End-Effector 2D Trajectory (Actual)', 'FontName', 'Times New Roman', 'FontSize', 12);
            cbar = colorbar;
            ylabel(cbar, 'Time (s)', 'FontName', 'Times New Roman', 'FontSize', 10);
            
            % 2D desired vs actual comparison
            subplot(1, 2, 2);
            plot(obj.O_his(1, :), obj.O_his(2, :), 'b-', 'LineWidth', 2); hold on;
            if ~isempty(obj.Odes_his)
                plot(obj.Odes_his(1, :), obj.Odes_his(2, :), 'r--', 'LineWidth', 2);
                legend('Actual', 'Desired', 'Location', 'best', 'FontName', 'Times New Roman');
            end
            grid on;
            axis equal;
            xlabel('X (mm)', 'FontName', 'Times New Roman', 'FontSize', 11);
            ylabel('Y (mm)', 'FontName', 'Times New Roman', 'FontSize', 11);
            title('End-Effector 2D Trajectory Comparison', 'FontName', 'Times New Roman', 'FontSize', 12);
        end
        
        function saveResults(obj, filename)
            % Save simulation results to .mat file (and optionally Excel)
            if nargin < 2
                filename = obj.plotConfig.saveFilename;
            end
            
            robot = obj.robots{1};
            
            % Prepare data structure
            results = struct();
            results.dt = obj.dt;
            results.time = obj.time;
            results.q = robot.q_his;
            results.qdot = robot.qdot_his;
            results.qddot = robot.qddot_his;
            results.u = robot.u_his;
            results.qdes = obj.qdes_his;
            results.qdotdes = obj.qdotdes_his;
            results.O = obj.O_his;
            results.Odes = obj.Odes_his;
            results.mode = obj.mode;
            
            % Save robot parameters for reconstruction
            results.robot = struct();
            results.robot.a = robot.a;
            results.robot.d = robot.d;
            results.robot.alpha = robot.alpha;
            results.robot.jointType = robot.jointType;
            results.robot.visual = robot.visual;
            
            % Save to .mat file
            save(filename, 'results');
            fprintf('Results saved to %s\\n', filename);
            
            % Optionally save to Excel if requested
            if isfield(obj.plotConfig, 'saveExcel') && obj.plotConfig.saveExcel
                [~, name, ~] = fileparts(filename);
                excelFile = [name '.xlsx'];
                obj.saveToExcel(excelFile, results);
            end
        end
        
        function saveToExcel(obj, filename, results)
            % Save results to Excel file
            t = (results.dt:results.dt:size(results.q, 2)*results.dt)';
            
            % Prepare data table
            n = size(results.q, 1);
            varNames = {'Time'};
            data = t;
            
            for i = 1:n
                varNames{end+1} = sprintf('q%d', i);
                data = [data results.q(i, :)'];
            end
            for i = 1:n
                varNames{end+1} = sprintf('qdot%d', i);
                data = [data results.qdot(i, :)'];
            end
            for i = 1:n
                varNames{end+1} = sprintf('u%d', i);
                data = [data results.u(i, :)'];
            end
            if ~isempty(results.qdes) && size(results.qdes, 2) == size(results.q, 2)
                for i = 1:n
                    varNames{end+1} = sprintf('qdes%d', i);
                    data = [data results.qdes(i, :)'];
                end
            end
            
            % Write to Excel
            T = array2table(data, 'VariableNames', varNames);
            writetable(T, filename);
            fprintf('Results saved to %s\n', filename);
        end
    end
    
    methods(Static)
        function loadAndPlot(filename, varargin)
            % Load and plot/visualize results from saved file
            % Usage: Simulation.loadAndPlot('sim_results.mat')
            %        Simulation.loadAndPlot('sim_results.mat', 'plot', true, 'animate', false)
            
            p = inputParser;
            addParameter(p, 'plot', true, @islogical);
            addParameter(p, 'animate', false, @islogical);
            parse(p, varargin{:});
            
            % Load data
            data = load(filename);
            results = data.results;
            
            fprintf('Loaded simulation results from %s\n', filename);
            fprintf('  Duration: %.2f s\n', results.time);
            fprintf('  Samples: %d\n', size(results.q, 2));
            
            % Plot results
            if p.Results.plot
                tf = size(results.q, 2) * results.dt;
                t = results.dt:results.dt:tf;
                
                % Plot q
                if ~isempty(results.q)
                    Simulation.plotVariableStatic(t, results.q, results.qdes, 'q', 'Angle (rad)');
                end
                
                % Plot qdot
                if ~isempty(results.qdot)
                    Simulation.plotVariableStatic(t, results.qdot, results.qdotdes, '\\dot{q}', 'Velocity (rad/s)');
                end
                
                % Plot u
                if ~isempty(results.u)
                    Simulation.plotVariableStatic(t, results.u, [], 'u', 'Torque (N$\\cdot$mm)');
                end
            end
            
            % Animate robot (if requested)
            if p.Results.animate && isfield(results, 'robot')
                Simulation.animateFromData(results);
            elseif p.Results.animate
                warning('Robot parameters not found in saved file. Animation requires robot parameters.');
            end
        end
        
        function animateFromData(results)
            % Animate robot motion from saved results
            % Create robot from saved parameters
            robot = Manipulator(results.robot.a, results.robot.d, ...
                results.robot.alpha, results.robot.jointType);
            robot.visual = results.robot.visual;
            
            % Create figure for animation
            fig = figure('Name', 'Simulation Playback', 'Position', [100, 100, 1000, 800]);
            ax = axes('Parent', fig, 'Position', [0.05 0.1 0.9 0.85]);
            grid(ax, 'on');
            xlabel(ax, 'X (mm)');
            ylabel(ax, 'Y (mm)');
            zlabel(ax, 'Z (mm)');
            view(ax, 3);
            rotate3d(ax, 'on');
            hold(ax, 'on');
            
            % Time controls
            uicontrol('Style', 'text', 'String', 'Time: 0.00 s', ...
                'Units', 'normalized', 'Position', [0.4 0.02 0.2 0.04], ...
                'Tag', 'timeText', 'FontSize', 10, 'HorizontalAlignment', 'center');
            
            % Animation loop
            nSamples = size(results.q, 2);
            fprintf('Animating %d samples (%.2f seconds)...\n', nSamples, results.time);
            
            for i = 1:nSamples
                % Update robot configuration
                robot.q = results.q(:, i);
                
                % Draw robot
                cla(ax);
                robot.draw(ax);
                
                % Update time display
                t = i * results.dt;
                timeText = findobj(fig, 'Tag', 'timeText');
                if ~isempty(timeText)
                    set(timeText, 'String', sprintf('Time: %.2f s', t));
                end
                
                % Real-time playback
                drawnow;
                if i < nSamples
                    pause(results.dt);
                end
            end
            
            fprintf('Animation complete.\n');
        end
        
        function plotVariableStatic(t, data, dataRef, varName, yLabel)
            % Static plotting method for loaded data
            n = size(data, 1);
            figure('Name', sprintf('Joint %s', varName), 'Position', [10, 10, 1000, 800]);
            for i = 1:n
                subplot(2, 3, i);
                plot(t, data(i, :), 'b', 'LineWidth', 1.5); hold on;
                if ~isempty(dataRef) && size(dataRef, 2) == size(data, 2)
                    plot(t, dataRef(i, :), 'r--', 'LineWidth', 1.5);
                    legend('Actual', 'Desired', 'Location', 'best');
                end
                grid on;
                xlim([0 max(t)]);
                title(sprintf('$%s_%d (t)$', varName, i), 'Interpreter', 'latex', ...
                    'FontName', 'Times New Roman', 'FontSize', 9);
                xlabel('Time (s)', 'FontName', 'Times New Roman', 'FontSize', 9);
                ylabel(yLabel, 'interpreter', 'latex', ...
                    'FontName', 'Times New Roman', 'FontSize', 9);
            end
        end
    end
end
