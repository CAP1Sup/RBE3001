robot = Robot();

move_time = 3;
robot.set_joint_vars([45,0,0,-45], 3*1000);
coeff = calc_t_coeff(robot, [200, 0, 100, 0], move_time);
robot.run_trajectory(coeff, move_time, true)


% Conveience function to generate the task space trajectory coefficients
function coeff = calc_t_coeff(robot, desired_pose, move_time)
    traj = Traj_Planner();
    joint_data = robot.read_joint_vars(true, false);
    fks = robot.joints2fk(joint_data(1,:));
    coeff = zeros(4,4);
    for index = 1:3 % X,Y,Z are easy
        coeff(index, :) = transpose(traj.cubic_traj([0; fks(index,4,4); 0], [move_time; desired_pose(1, index); 0]));
    end
    % Alpha is a little harder
    % Get the Euler angles from the end effector matrix
    % Z should be the same as q1 and Y should be the alpha of the effector
    % X should always be -90 deg... J3 Z axis is rotated to be horizontal
    eulZYX = rad2deg(tform2eul(fks(:,:,4))); % [Z,Y,X]
    coeff(4,:) = transpose(traj.cubic_traj([0; eulZYX(1,2); 0], [move_time; desired_pose(1,4); 0]));
end