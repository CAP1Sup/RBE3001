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
axis([-50, 100, -100, 100, -50, 400]);

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

% Generate the trajectory
traj_coef = calc_quintic_t_coeff(robot, pose2, travelTime);

% Continously monitor the joint positions
tic;
while toc < travelTime
    % Calculate the new position
    time = toc;
    x = polyval(flip(traj_coef(1,:), 2), time);
    y = polyval(flip(traj_coef(2,:), 2), time);
    z = polyval(flip(traj_coef(3,:), 2), time);
    alpha = polyval(flip(traj_coef(4,:), 2), time);
    joint_vals = robot.task2ik([x,y,z,alpha]);

    % Move there
    robot.set_joint_vars(joint_vals, 0);
    pause(0.01)

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

% Conveience function to generate the task space quintic trajectory coefficients
function coeff = calc_quintic_t_coeff(robot, desired_pose, move_time)
    traj = Traj_Planner();
    joint_data = robot.read_joint_vars(true, false);
    fks = robot.joints2fk(joint_data(1,:));
    coeff = zeros(4,6);
    for index = 1:3 % X,Y,Z are easy
        coeff(index, :) = transpose(traj.quintic_traj([0; fks(index,4,4); 0; 0], [move_time; desired_pose(1, index); 0; 0]));
    end
    % Alpha is a little harder
    % Get the Euler angles from the end effector matrix
    % Z should be the same as q1 and Y should be the alpha of the effector
    % X should always be -90 deg... J3 Z axis is rotated to be horizontal
    eulZYX = rad2deg(tform2eul(fks(:,:,4))); % [Z,Y,X]
    coeff(4,:) = transpose(traj.quintic_traj([0; eulZYX(1,2); 0; 0], [move_time; desired_pose(1,4); 0; 0]));
end