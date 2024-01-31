%% Setup robot
travelTime = 10; % Defines the travel time
model = Model(); % Creates model object
waypoint = [45, -55, -50,  90;
            20,  20,  20,  20;
            10, -20,  15, -12;
           -80, -70,   5,   0;
           -30,  30,  10,  -45]; % Give the angle for each position
home = [0,0,0,0]; % Home 0,0,0,0 position
trialNum = 1; %% SET THIS NUMBER FOR EACH INDIVIDUAL TRIAL

%% Move the robot back to the home
model.robot.set_joint_vars(home, 3000);
pause(3);

%% Create the plot, perform the movement
robotPlot = model.new_arm_plot();
model.robot.set_joint_vars(waypoint(trialNum, :), travelTime*1000);

% Start the timer
tic;
while toc <= travelTime
    joint_pos = model.robot.read_joint_vars(true, false);
    model.plot_arm(joint_pos(1, :), robotPlot);
    xlabel('x pos (mm)')
    ylabel('y pos (mm)')
    zlabel('z pos (mm)')
    grid("on")
    title("3D arm model at position " + trialNum)
    set(gca,'fontsize',16)
end
