model = Model();
travelTime = 3; % s
home_pos = [0,0,0,0];

% Move to the home position
model.robot.set_joint_vars(home_pos,travelTime*1000);
pause(travelTime)

% 3D plot the arm
figure(1)
[robotPlot, xQPlot, yQPlot, zQPlot] = model.new_arm_plots();
model.plot_arm(home_pos, robotPlot, xQPlot, yQPlot, zQPlot);
axis([-100, 400, -300, 300, -10, 400]);
view(0,0);

% Loop through this 3 times
for interation = 1:3
    % Make a new figure and have the user pick a point
    figure(2)
    title("XZ Point Selection")
    axis([0, 400, -10, 400]);
    xlabel("X Pos (mm)")
    ylabel("Z Pos (mm)")
    [x,z] = ginput(1);
    close 2
    
    % Point to go to
    coords = [x, 0, z, 0];
    
    % Calculate the inverse kinematics
    newton_ik_angles = model.robot.numerical_task2ik(coords);

    % Move the specified location
    model.robot.set_joint_vars(newton_ik_angles(end, :), travelTime*1000)
    
    % Plot the IK angles over time
    for angle_index = 1:length(newton_ik_angles)
        model.plot_arm(newton_ik_angles(angle_index,:), robotPlot, xQPlot, yQPlot, zQPlot);
        pause(travelTime / length(newton_ik_angles))
    end % IK angle plotting
end % 3x iteration