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
currPos = robot.read_joint_vars(true , true);
joint_vel = [0,0,0,0,0];
joint_pos = [0,0,0,0,0];
joint_value = [0,0,0,0,0; 0,0,0,0,0];
time = [0];
for trial = currTrial:maxTrials
    coeffs = calc_quintic_t_coeff(robot,location(trial+1,:), travelTime);
    new_joint_value = robot.run_trajectory_pos_vel(coeffs, travelTime, true);

    new_joint_vel = robot.read_joint_vars(false , true);
    new_joint_vel = [new_joint_vel(2,:)];
    
    
    if exist("prev_joint_value", "var")
        new_joint_value(:,1) = new_joint_value(:,1) + max(prev_joint_value(:,1)) + 0.1;
    end
    if exist("joint_pos", "var")
        joint_value = [joint_value; new_joint_value];
    else
        joint_value = new_joint_value;
    end
    prev_joint_value = new_joint_value;

end % for

    for i = 1:length(joint_value)
            if mod(i,2) == 0
            joint_vel = [joint_vel; joint_value(i,:)];
            else
            joint_pos = [joint_pos; joint_value(i,:)];
            time = [time; max(joint_pos(:,1))];
            end
    end
time = time(3:end,1);               % gets rid of first row being 0
joint_vel = joint_vel(3:end,2:end); % gets rid of first row being 0
joint_pos = joint_pos(3:end,2:end); % gets rid of first row being 0

x = joint_pos(14,1:end);
y = transpose(joint_vel(14,1:end));
disp(x);
disp(y);
TSvel = vel2fdk(x,y);


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