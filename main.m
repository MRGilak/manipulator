function main
    clc; close all;

    %% DH parameters
    l1 = 396; l2 = 324; l3 = 220; l4 = 123; l5 = 165; l6 = 50;

    a     = [0, l2, 0, 0, 0, 0];
    d     = [l1, 0, 0, l3+l4, 0, l5+l6];
    alpha = deg2rad([90, 0, -90, -90, 90, 0]);

    jointType = ['R','R','R','R','R','R'];

    robot = Manipulator(a,d,alpha,jointType);
    
    % Configure link visualization
    robot.visual.linkLengths = [l1, l2, l3, l4, l5, l6];
    
    % Link type: 'z' = extends along z-axis of frame (i-1)
    %            'xy' = extends in x-y plane of frame (i-1), rotates with q(i)
    robot.visual.linkType = {'z', 'xy', 'xy', 'z', 'xy', 'z'};
    
    % Link offset: where each link starts along the z-axis of frame (i-1)
    robot.visual.linkOffset = [0, 0, 0, l3, 0, l5];
    
    % Theta offset: additional rotation for 'xy' type links (in radians)
    % Example: pi/2 for 90 degrees rotation even when q(i) = 0
    robot.visual.linkThetaOffset = [0, 0, pi/2, 0, -pi/2, 0];  % Link 3 rotated 90° from x-axis

    %% Figure
    fig = figure('Name','Manipulator','NumberTitle','off','Position',[10 10 1000 800]);
    ax = axes('Parent',fig,'Position',[0.05 0.05 0.65 0.9]);
    grid(ax,'on');
    
    xlabel(ax,'X');
    ylabel(ax,'Y');
    zlabel(ax,'Z');
    
    view(ax,3);
    camproj(ax,'perspective');
    rotate3d(ax,'on');

    robot.draw(ax);
    
    % Set view after drawing so axis auto-fits the robot
    axis(ax,'vis3d');
    axis(ax,'equal');

    %% Sliders
    for i = 1:6
        uicontrol('Style','text',...
            'String',sprintf('Joint %d',i),...
            'Units','normalized',...
            'Position',[0.78 0.90-0.12*i 0.08 0.03],...
            'HorizontalAlignment','left');
        uicontrol('Style','slider',...
            'Min',-pi,'Max',pi,'Value',0,...
            'Units','normalized',...
            'Position',[0.78 0.87-0.12*i 0.18 0.04],...
            'Callback',@(src,~) sliderCB(i,src.Value));
    end

    %% Nested callback 
    function sliderCB(idx,val)
        robot.q(idx) = val;
        robot.updateGraphics();
    end
end
