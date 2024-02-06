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
    ik_pose = robot.task2ik(location(trial+1,:));
    coeffs = calc_j_coeff(robot, ik_pose, travelTime);
    new_joint_pos = robot.run_trajectory(coeffs, travelTime);
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
title('Joint Angles Cubic Trajectory of a Triangle From Inverse Kinematics')
xlabel('Time (s)')
ylabel('Pos (deg)')
axis([0 max(joint_pos(:,1)) -90 90])
set(gca,'fontsize',16);




% Conveience function to generate the joint trajectory coefficients
function coeff = calc_j_coeff(robot, desired_ang, move_time)
    traj = Traj_Planner();
    joint_data = robot.read_joint_vars(true, false);
    current_ang = joint_data(1,:);
    coeff = zeros(4,4);
    for index = 1:4
        coeff(index, :) = transpose(traj.cubic_traj([0; current_ang(1,index); 0], [move_time; desired_ang(1, index); 0]));
    end
end