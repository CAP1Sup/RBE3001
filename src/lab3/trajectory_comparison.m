robot = Robot();

%% Load all of the data
load("no_interp_angles.mat", "positionData");
no_interp_angles = positionData;
load("joint_space_angles.mat", "joint_pos");
joint_space_angles = joint_pos;
load("task_space_angles.mat", "joint_pos");
task_space_angles = joint_pos;

%% Generate the end effector positions
no_interp_coords = zeros(length(no_interp_angles), 3);
for index = 1:length(no_interp_angles)
    fks = robot.joints2fk(no_interp_angles(index, 2:5));
    no_interp_coords(index,:) = [fks(1,4,4), fks(2,4,4), fks(3,4,4)];
end % no_interp_angles

joint_space_coords = zeros(length(joint_space_angles), 3);
for index = 1:length(joint_space_angles)
    fks = robot.joints2fk(joint_space_angles(index, 2:5));
    joint_space_coords(index,:) = [fks(1,4,4), fks(2,4,4), fks(3,4,4)];
end % joint_space_angles

task_space_coords = zeros(length(task_space_angles), 3);
for index = 1:length(task_space_angles)
    fks = robot.joints2fk(task_space_angles(index, 2:5));
    task_space_coords(index,:) = [fks(1,4,4), fks(2,4,4), fks(3,4,4)];
end % task_space_angles

%% Scatter plot the points
figure(1)
title("End Effector Position Using Different Trajectory Methods")
xlabel("X (mm)")
ylabel("Y (mm)")
zlabel("Z (mm)")
hold on
point_size = 25;
scatter3(no_interp_coords(:,1), no_interp_coords(:,2), no_interp_coords(:,3), point_size, "red")
scatter3(joint_space_coords(:,1), joint_space_coords(:,2), joint_space_coords(:,3), point_size, "blue")
scatter3(task_space_coords(:,1), task_space_coords(:,2), task_space_coords(:,3), point_size, "green")
legend("No interpolation", "Joint space trajectory", "Task space trajectory", 'Location', 'northwest')
hold off