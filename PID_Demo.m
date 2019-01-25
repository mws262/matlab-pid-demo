function PID_Demo
%% PID controller demo for MAE 3260, Spring 2019
% Simulation and plotting is done in realtime and settings may be altered
% on the fly. Should not need to edit this file for basic functions.
%
% Run to launch GUI. Initially spring-mass-damper dynamics with no
% controller active.
%
% Menu on the right can change the uncontrolled system to:
% spring-mass-damper, spring-mass, mass-damper, or mass.
%
% Controls are activated by changing the gains from zero with the sliders
% at the bottom. By default, the controller setpoint is changed by clicking
% the plot. Menu on the right can change the controller's setpoint to a:
% square wave, triangle wave, sawtooth wave, or sine wave.
%
% Matt Sheen 2019
%

%% Parameters
% Parameters of uncontrolled system.
m = 1;
c = 3;
k = 30;

initialPos = 5;
initialVel = 0;

% Initial controller parameters.
p = 0; % Proportional gain (spring).
i = 0; % Integral gain.
d = 0; % Derivative gain (damper).
maxP = 200; % Maximum gain allowed by slider.
maxI = 200;
maxD = 100;

tend = 5; % Window of simulation time being shown.

% Target of controller.
targetPos = 4;
targetVel = 0; % Not changed (so far anyway).

%% GUI setup:
fig = figure(101);
fig.Name = 'PID Example';
fig.MenuBar = 'none';
fig.Position(3:4) = [800,600];

plotAx = subplot(4,4,[1,2,3,5,6,7,9,10,11]); % For a time history plot of the result.
plotAx.ButtonDownFcn = @setpointClickFcn;
plotAx.Box = 'on';
hold on;
positionTrace = plot(0,0, 'LineWidth', 2);
currentPositionPt = plot(0,0,'.r', 'MarkerSize', 30);
setpointLine = plot([0, tend], [targetPos, targetPos], ':k', 'LineWidth', 2); % Show the target of the controller.
hold off;
axis([0,tend, -10, 10]);

% Bottom 1/4 is left open for slider bars (P, I, & D gains).
pSlider = uicontrol(fig, 'Style', 'slider');
pSlider.Units = 'normalized';
pSlider.Position(3) = 0.8; % Make it nearly the width of the figure.
pSlider.Position(2) = 0.19; % Vertically space out sliders.
pSlider.Callback = @pCallback;
pSliderLabel = uicontrol(fig, 'Style', 'text', 'String', 'Proportional gain: 0', 'Units', 'normalized');
pSliderLabel.Position = [0.85, 0.15, 0.1, 0.08];

iSlider = uicontrol(fig, 'Style', 'slider');
iSlider.Units = 'normalized';
iSlider.Position(3) = 0.8; % Make it nearly the width of the figure.
iSlider.Position(2) = 0.12;
iSlider.Callback = @iCallback;
iSliderLabel = uicontrol(fig, 'Style', 'text', 'String', 'Integral gain: 0', 'Units', 'normalized');
iSliderLabel.Position = [0.85, 0.08, 0.1, 0.08];

dSlider = uicontrol(fig, 'Style', 'slider');
dSlider.Units = 'normalized';
dSlider.Position(3) = 0.8; % Make it nearly the width of the figure.
dSlider.Position(2) = 0.05;
dSlider.Callback = @dCallback;
dSliderLabel = uicontrol(fig, 'Style', 'text', 'String', 'Derivative gain: 0', 'Units', 'normalized');
dSliderLabel.Position = [0.85, 0.01, 0.1, 0.08];

% Select which dynamics should be simulated currently.
dynamicsSelector = uicontrol(fig,'Style','popupmenu');
dynamicsSelector.Units = 'normalized';
dynamicsSelector.Position = [0.72 0.8 0.25 0.1];
dynamicsSelector.String = {'Spring-Mass-Damper','Spring-Mass','Mass-Damper', 'Mass only'};
dynamicsSelector.Callback = @dynamicsSelectorFcn;
currentDynamics = 1; % 1,2,3 corresponding to the selector values.
dynamicsLabel = uicontrol(fig, 'Style', 'text', 'String', 'Dynamics (w/o controller)', 'Units', 'normalized');
dynamicsLabel.Position = [0.72, 0.9, 0.25, 0.03];

% Select the sort of setpoint being tracked. Defaults to click the plot to
% select.
setpointSelector = uicontrol(fig,'Style','popupmenu');
setpointSelector.Units = 'normalized';
setpointSelector.Position = [0.72 0.65 0.25 0.1];
setpointSelector.String = {'Click plot','Square wave','Triangle wave', 'Sawtooth wave', 'Sine wave'};
setpointSelector.Callback = @setpointSelectorCallback;
currentSetpointMode = 1; % 1,2,3,4,5 corresponding to the selector values.
setpointLabel = uicontrol(fig, 'Style', 'text', 'String', 'Controller setpoint', 'Units', 'normalized');
setpointLabel.Position = [0.72, 0.75, 0.25, 0.03];

%% Realtime integration and plotting loop.
currentState = [initialPos, initialVel, 0];
bufferSize = 200;
timeBuffer = zeros(bufferSize,1);
positionBuffer = ones(bufferSize,1) * initialPos(1);
tic;
currTime = toc;
prevTime = currTime;
while (ishandle(fig)) % Dies when window is closed.
    currTime = toc;
    
    % What signal are we tracking?
    switch currentSetpointMode
        case 1
            % No need to change here.
        case 2
            targetPos = 5*square(currTime);
        case 3
            targetPos = 5*sawtooth(currTime,0.5);
        case 4
            targetPos = 5*sawtooth(currTime);
        case 5
            targetPos = 5*sin(currTime);
    end
    
    % RK4 integration, as fast as computer is able to run this loop.
    dt = currTime - prevTime;
    k_1 = dt * springMassDamper(0, currentState);
    k_2 = dt * springMassDamper(0, currentState + k_1/2);
    k_3 = dt * springMassDamper(0, currentState + k_2/2);
    k_4 = dt * springMassDamper(0, currentState + k_3);
    currentState = currentState + (k_1 + 2*k_2 + 2*k_3 + k_4)/6;
    prevTime = currTime;
    
    % Only update the plot buffer if there's been enough change in time for
    % it to matter.
    if (currTime - timeBuffer(end) > tend / bufferSize)
        % Shift and update buffers.
        timeBuffer = circshift(timeBuffer, -1);
        timeBuffer(end) = currTime;
        positionBuffer = circshift(positionBuffer, -1);
        positionBuffer(end) = currentState(1);
        positionTrace.XData = timeBuffer;
        % Update plot data and re-draw.
        positionTrace.YData = positionBuffer;
        currentPositionPt.XData = timeBuffer(end);
        currentPositionPt.YData = positionBuffer(end);
        plotAx.XLim = [timeBuffer(1), timeBuffer(end)];
        setpointLine.XData = [timeBuffer(1), timeBuffer(end)];
        setpointLine.YData = [targetPos, targetPos];
        drawnow;
    end
end

%% Dynamics to integrate. Includes both system and controller.
    function qdot = springMassDamper(~,q)
        position = q(1);
        velocity = q(2);
        error_integral = q(3); % State augmented to keep track of the integral of an error term also for the integral controller.
        
        controlForce = p * (targetPos - position) + d * (targetVel - velocity) - i * (error_integral);
        
        qdot = zeros(3,1);
        qdot(1) = velocity;
        switch currentDynamics
            case 1 % SMD
                qdot(2) = -c/m * velocity - k/m * position + controlForce;
            case 2 % SM
                qdot(2) = -k/m * position + controlForce;
            case 3 % MD
                qdot(2) = -c/m * velocity + controlForce;
            case 4 % M
                qdot(2) = controlForce;
        end
        qdot(3) = position - targetPos;
    end

%% Callback functions for user interaction.
    function pCallback(src, ~)
        p = src.Value * maxP;
        pSliderLabel.String = strcat('Proportional gain: ', num2str(round(p, 1))); % Update text label.
    end
    function iCallback(src, ~)
        if src.Value > 0
            currentState(3) = i/(src.Value * maxI) * currentState(3); % Scale current accumulator value so there isn't a jump in integral term force.
        end
        i = src.Value * maxI;
        iSliderLabel.String = strcat('Integral gain: ', num2str(round(i, 1)));
    end
    function dCallback(src, ~)
        d = src.Value * maxD;
        dSliderLabel.String = strcat('Derivative gain: ', num2str(round(d, 1)));
    end

    function setpointClickFcn(src, ~)
        if currentSetpointMode == 1 % Only if the click-to-set mode is selected.
            clickY = src.CurrentPoint(1,2);
            setpointLine.YData = [clickY, clickY];
            targetPos = clickY;
            currentState(3) = 0; % Clear integrator when setpoint is changed.
        end
    end
    function dynamicsSelectorFcn(src, ~)
        currentDynamics = src.Value;
    end
    function setpointSelectorCallback(src, ~)
        currentSetpointMode = src.Value;
    end
end