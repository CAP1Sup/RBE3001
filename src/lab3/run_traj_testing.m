robot = Robot();
move_time = 3;
joint_angs = [-10, -10, -10, -10];

robot.set_joint_vars([0, 0, 0, 0], move_time*1000);
pause(move_time)
coeffs = calc_j_coeff(robot, joint_angs, move_time);
robot.run_trajectory(coeffs, move_time)




% Conveience function to generate the joint trajectory coefficients
function coeff = calc_j_coeff(robot, desired_ang, move_time)
    traj = Traj_Planner();
    joint_data = robot.read_joint_vars(true, false);
    current_ang = joint_data(1,:)
    coeff = zeros(4,4);
    for index = 1:4
        coeff(index, :) = transpose(traj.cubic_traj([0; current_ang(1,index); 0], [move_time; desired_ang(1, index); 0]));
    end
end