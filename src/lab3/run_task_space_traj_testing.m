clear;
%% SETUP
robot = Robot();

location = [200,  0,   100,  0;
            100,  200, 350, -90;
            50,  -70,  300, -45;
            200,  0,   100,  0];

travelTime = 3; % seconds
maxTrials = 3; %% SET THIS NUMBER FOR EACH INDIVIDUAL TRIAL
currTrial = 1; % initalize trial start

%% CODE

robot.set_joint_vars([0, 0, 0, 0], travelTime*1000);
pause(travelTime)
ik_pose = robot.task2ik(location(currTrial,:));
robot.set_joint_vars(ik_pose,travelTime*1000);
pause(travelTime);

for trial = currTrial:currTrial+2
    coeffs = calc_t_coeff(robot,location(trial+1,:), travelTime);
    new_joint_pos = robot.run_trajectory(coeffs, travelTime, true);
    if exist("prev_joint_pos", "var")
        new_joint_pos(:,1) = new_joint_pos(:,1) + max(prev_joint_pos(:,1));
    end
    if exist("joint_pos", "var")
        joint_pos = [joint_pos; new_joint_pos];
    else
        joint_pos = new_joint_pos;
    end
    prev_joint_pos = new_joint_pos;
end % for


figure(1)
plot(joint_pos(:,1), joint_pos(:,2));
hold on
plot(joint_pos(:,1), joint_pos(:,3));
plot(joint_pos(:,1), joint_pos(:,4));
plot(joint_pos(:,1), joint_pos(:,5));
hold off
legend('Base angle', 'Shoulder Angle', 'Elbow Angle','Wrist Angle', 'Location', 'northwest')
title('Joint Angles Cubic Trajectory of a Triangle From Inverse Kinematics in Task Space')
xlabel('Time (s)')
ylabel('Pos (deg)')
axis([0 max(joint_pos(:,1)) -100 100])
set(gca,'fontsize',16);






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