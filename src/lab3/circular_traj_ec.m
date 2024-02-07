clear;
robot = Robot();
traj_gen = Traj_Planner();

setup_move_time = 2; % s
circle_time = 8; % s

% Circle size
x_max = 20; % mm
y_max = 100; % mm
z_max = 100; % mm

% Circle offset
x_offset = 200; % mm
z_offset = 200; % mm

% Move to the home position
robot.set_joint_vars([0,0,0,0], setup_move_time*1000);
pause(setup_move_time);

% Calculate the trajectory
angle_coeffs = traj_gen.quintic_traj([0, 0, 0, 0],[circle_time, 2*pi, 0, 0]);
flipped_coeffs = flip(angle_coeffs, 1);

% Move to the initial coordinate
x = x_max * sin(polyval(flipped_coeffs, 0)) + x_offset;
y = y_max * cos(polyval(flipped_coeffs, 0));
z = z_max * sin(polyval(flipped_coeffs, 0)) + z_offset;
joints = robot.task2ik([x,y,z,0]);
robot.set_joint_vars(joints, setup_move_time*1000);
pause(setup_move_time);

% Seed the position matrix
currPos = robot.read_joint_vars(true, false);
pos_matrix = robot.joints2fk(currPos(1,:));
pos_matrix = transpose(pos_matrix(1:3,4,4));
pos_matrix = [pos_matrix,0,0];

% Move through the trajectory
tic;
while toc <= circle_time
    time = toc;
    x = x_max * sin(polyval(flipped_coeffs, time)) + x_offset;
    y = y_max * cos(polyval(flipped_coeffs, time));
    z = z_max * sin(polyval(flipped_coeffs, time)) + z_offset;
    joints = robot.task2ik([x,y,z,0]);
    robot.set_joint_vars(joints, 0);
    joint_vars = robot.read_joint_vars(true, false);
    if exist("joint_angles", "var")
        joint_angles = [joint_angles; time, joint_vars(1,:)];
    else
        joint_angles = [time, joint_vars(1,:)];
    end
    pause(0.01); % second to prevent motors from choking
end % toc <= move_time

end_eff_coords = zeros(length(joint_angles), 3);
for index = 1:length(joint_angles)
    fks = robot.joints2fk(joint_angles(index, 2:5));
    end_eff_coords(index,:) = [fks(1,4,4), fks(2,4,4), fks(3,4,4)];
end % task_space_angles

%% Scatter plot the points
figure(1)
hold on
title("End Effector Position During Circular Quintic Trajectory")
xlabel("X (mm)")
ylabel("Y (mm)")
zlabel("Z (mm)")
axis([0,300,-150,150,0,300])
scatter3(end_eff_coords(:,1), end_eff_coords(:,2), end_eff_coords(:,3), 25, "green")
hold off

%% Calculate the XYZ positions, velocities, and accelerations
% Position
for i = 2:length(joint_angles)
    tMatrix = robot.joints2fk(joint_angles(i,2:end));
    tMatrix = transpose(tMatrix(1:3,4,4));
    tMatrix = [tMatrix,0,0];
    pos_matrix = [pos_matrix; tMatrix];
end

% Velocity
for i = 2:length(pos_matrix)
    velx = (pos_matrix(i,1)-pos_matrix(i-1,1)) / (joint_angles(i,1)-joint_angles(i-1,1));
    vely = (pos_matrix(i,2)-pos_matrix(i-1,2)) / (joint_angles(i,1)-joint_angles(i-1,1));
    velz = (pos_matrix(i,3)-pos_matrix(i-1,3)) / (joint_angles(i,1)-joint_angles(i-1,1));

    if exist("vel_matrix", "var")
        vel_matrix = [vel_matrix; velx,vely,velz];
    else
        vel_matrix = [velx,vely,velz];
    end
end

% Acceleration
for i = 2:length(vel_matrix)
    accx = (vel_matrix(i,1)-vel_matrix(i-1,1)) / (joint_angles(i,1)-joint_angles(i-1,1));
    accy = (vel_matrix(i,2)-vel_matrix(i-1,2)) / (joint_angles(i,1)-joint_angles(i-1,1));
    accz = (vel_matrix(i,3)-vel_matrix(i-1,3)) / (joint_angles(i,1)-joint_angles(i-1,1));
    
    if exist("acc_matrix", "var")
        acc_matrix = [acc_matrix; accx,accy,accz];
    else
        acc_matrix = [accx,accy,accz];
    end      
end

%% Generate the graphs
figure(2)
plot(joint_angles(:,1), pos_matrix(:,1));
hold on
plot(joint_angles(:,1), pos_matrix(:,2));
plot(joint_angles(:,1), pos_matrix(:,3));
hold off
legend('X', 'Y', 'Z','Location', 'northwest')
title('Circular Traj - EF Position over Time')
xlabel('Time (s)')
ylabel('Pos (mm)')
axis([0 max(joint_angles(:,1)) -140 430])
set(gca,'fontsize',16);


figure(3)
plot(joint_angles(2:end,1), vel_matrix(:,1));
hold on
plot(joint_angles(2:end,1), vel_matrix(:,2));
plot(joint_angles(2:end,1), vel_matrix(:,3));
hold off
legend('X', 'Y', 'Z','Location', 'northwest')
title('Circular Traj - EF Velocity over Time')
xlabel('Time (s)')
ylabel('Velocity (mm/s)')
axis([0 max(joint_angles(:,1)) -250 250])
set(gca,'fontsize',16);

figure(4)
plot(joint_angles(3:end,1), acc_matrix(:,1));
hold on
plot(joint_angles(3:end,1), acc_matrix(:,2));
plot(joint_angles(3:end,1), acc_matrix(:,3));
hold off
legend('X', 'Y', 'Z','Location', 'northwest')
title('Circular Traj - EF Acceleration over Time')
xlabel('Time (s)')
ylabel('Accel (mm/s^2)')
axis([0 max(joint_angles(:,1)) -10000 10000])
set(gca,'fontsize',16);