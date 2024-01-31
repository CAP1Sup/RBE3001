%% Create a new instance of Robot (used for fk)
robot = Robot();
end_effector_pos = [0, 0, 0];
deg_step = 15;

total_pts = (180/deg_step + 1) * (360/deg_step + 1)^3;
current_pt = 0;
for j1 = -90:deg_step:90
for j2 = -180:deg_step:180
for j3 = -180:deg_step:180
for j4 = -180:deg_step:180
    fks = robot.joints2fk([j1, j2, j3, j4]);
    new_pos = [fks(1,4,4), fks(2,4,4), fks(3,4,4)];
    end_effector_pos = [end_effector_pos; new_pos];
    current_pt = current_pt + 1;
    "Point: " + current_pt + " of " + total_pts
end % j4
end % j3
end % j2
end % j1

%% Save the points
save("workspace_points.mat", "end_effector_pos")

%% Plot all of the points
close all
scatter3(end_effector_pos(:,1), end_effector_pos(:,2), end_effector_pos(:,3), 1)
title("Workspace Plot")