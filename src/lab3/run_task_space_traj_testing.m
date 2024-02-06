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
currPos = robot.read_joint_vars(true , false);
TMatrix = robot.joints2fk(currPos(1,:));
TMatrix = transpose(TMatrix(1:3,4,4));
TMatrix = [TMatrix,0,0];

for trial = currTrial:currTrial+2
    coeffs = calc_t_coeff(robot,location(trial+1,:), travelTime);
    new_joint_pos = robot.run_trajectory(coeffs, travelTime, true);
    if exist("prev_joint_pos", "var")
        new_joint_pos(:,1) = new_joint_pos(:,1) + max(prev_joint_pos(:,1)) + 0.1;
    end
    if exist("joint_pos", "var")
        joint_pos = [joint_pos; new_joint_pos];
    else
        joint_pos = new_joint_pos;
    end
    prev_joint_pos = new_joint_pos;
end % for


    for i = 2:length(joint_pos)

    tMatrix = robot.joints2fk(joint_pos(i,2:end));
    tMatrix = transpose(tMatrix(1:3,4,4));
    tMatrix = [tMatrix,0,0];
    TMatrix = [TMatrix; tMatrix];

    end

    for i = 2:length(TMatrix)
        
        velx = (TMatrix(i,1)-TMatrix(i-1,1)) / (joint_pos(i,1)-joint_pos(i-1,1));
        vely = (TMatrix(i,2)-TMatrix(i-1,2)) / (joint_pos(i,1)-joint_pos(i-1,1));
        velz = (TMatrix(i,3)-TMatrix(i-1,3)) / (joint_pos(i,1)-joint_pos(i-1,1));

        if exist("VelMatrix")
        VelMatrix= [VelMatrix; velx,vely,velz];
        else
        VelMatrix = [velx,vely,velz];
        end

    end


    for i = 2:length(VelMatrix)

        accx = (VelMatrix(i,1)-VelMatrix(i-1,1)) / (joint_pos(i,1)-joint_pos(i-1,1));
        accy = (VelMatrix(i,2)-VelMatrix(i-1,2)) / (joint_pos(i,1)-joint_pos(i-1,1));
        accz = (VelMatrix(i,3)-VelMatrix(i-1,3)) / (joint_pos(i,1)-joint_pos(i-1,1));

        if exist("AccMatrix")
        AccMatrix= [AccMatrix; accx,accy,accz];
        else
        AccMatrix = [accx,accy,accz];
        end      

    end






figure(1)

plot(joint_pos(:,1), joint_pos(:,2));
hold on
plot(joint_pos(:,1), joint_pos(:,3));
plot(joint_pos(:,1), joint_pos(:,4));
plot(joint_pos(:,1), joint_pos(:,5));
hold off
legend('Base angle', 'Shoulder Angle', 'Elbow Angle','Wrist Angle', 'Location', 'northwest')
title('Joint Angles Cubic Trajectory of a Triangle From Inverse Kinematics In Task Space')
xlabel('Time (s)')
ylabel('Pos (deg)')
axis([0 max(joint_pos(:,1)) -100 100])
set(gca,'fontsize',16);


figure(2)
plot(joint_pos(:,1), TMatrix(:,1));
hold on
plot(joint_pos(:,1), TMatrix(:,2));
plot(joint_pos(:,1), TMatrix(:,3));
hold off
legend('X position of EF', 'Y position of EF', 'Z position of EF','Location', 'northwest')
title('X,Y,Z positions of EF In Task Space')
xlabel('Time (s)')
ylabel('Pos (mm)')
axis([0 max(joint_pos(:,1)) -140 430])
set(gca,'fontsize',16);


figure(3)
plot(joint_pos(2:end,1), VelMatrix(:,1));
hold on
plot(joint_pos(2:end,1), VelMatrix(:,2));
plot(joint_pos(2:end,1), VelMatrix(:,3));
hold off
legend('X velocity of EF', 'Y velocity of EF', 'Z velocity of EF','Location', 'northwest')
title('X,Y,Z velocities of EF in Task Space')
xlabel('Time (s)')
ylabel('Velocity (mm/s)')
axis([0 max(joint_pos(:,1)) -250 250])
set(gca,'fontsize',16);

figure(4)
plot(joint_pos(3:end,1), AccMatrix(:,1));
hold on
plot(joint_pos(3:end,1), AccMatrix(:,2));
plot(joint_pos(3:end,1), AccMatrix(:,3));
hold off
legend('X acceleration of EF', 'Y acceleration of EF', 'Z acceleration of EF','Location', 'northwest')
title('X,Y,Z accelerations of EF In Task Space')
xlabel('Time (s)')
ylabel('Accel (mm/s^2)')
axis([0 max(joint_pos(:,1)) -10000 10000])
set(gca,'fontsize',16);


%% Save the joint angles over time
save("task_space_angles", "joint_pos");


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