robot = Robot();
travelTime = 3; % s

% Singularities happen where the robot arm has to "flip"
% These two poses should push the robot through a singularity
pose1 = [50, 0, 200, 0];
pose2 = [-50, 0, 200, 0];

% Create the end effector graph
ee_pos_fig = figure(1);
ee_pos_plot = scatter3(0,0,0);
title("End Effector Position Over Time")
xlabel("X Pos (mm)")
ylabel("Y Pos (mm)")
zlabel("Z Pos (mm)")
axis([-300, 300, -300, 300, -50, 400]);

% Create the determinant graph
det_fig = figure(2);
det_plot = plot(0,0);
title("Jacobian Det Over Time")
xlabel("Time (s)")
ylabel("Jacobian determinant")
axis([0, travelTime, 0, 1000]);

% Data accumulators
ee_pos = pose1(1:3);
det_over_time = [0, 0];

% Move to the first pose
robot.set_joint_vars(robot.task2ik(pose1), travelTime*1000);
pause(travelTime)

% Command the robot to move to the 2nd pose
robot.set_joint_vars(robot.task2ik(pose2), travelTime*1000);

% Continously monitor the joint positions
tic;
while toc < travelTime
    % Get the joint positions
    joint_vars = robot.read_joint_vars(true, false);
    joint_pos = joint_vars(1,:);

    % Calculate the Jacobian
    jacobian = robot.get_jacobian(joint_pos);

    % Check if we're approaching a singularity
    % Should throw an error if we are
    % Also returns the determinant of the upper 3x3
    jacob_det = robot.prevent_singularity(jacobian);

    % Calculate the fks
    fks = robot.joints2fk(joint_pos);

    % Accumulate the data
    ee_pos = [ee_pos; fks(1,4,4), fks(2,4,4), fks(3,4,4)];
    det_over_time = [det_over_time; toc, jacob_det];

    % Update the plots
    set(ee_pos_plot, "XData", ee_pos(:,1), "YData", ee_pos(:,2), "ZData", ee_pos(:,3));
    set(det_plot, "XData", det_over_time(:,1), "YData", det_over_time(:,2));
    axis([0, travelTime, 0, max(det_over_time(:,2))]);
end % 2nd pose move time