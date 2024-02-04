%% Choose poses for robot to move between
poses = [200,  0,   100,  0;
         100,  200, 300, -90;
         150, -150, 10,   90];
move_time = 3; % seconds

%% Create model and robot
model = Model();
model.robot.writeTime(move_time);

% Seed the fks variable with the fks from the first pose
% Maybe preallocate?
jointsVsT = [model.robot.task2ik(pose(1))];
fksVsT = [model.robot.joints2fk(jointsVsT(1))];
times = [0];

% Start timer
tic;

% Loop through the poses
for pose = transpose(poses)
    model.robot.set_joint_vars(task2ik(pose));

    move_start_time = toc;
    while toc <= move_start_time + move_time
        % Calculate the values
        joint_vars = model.robot.read_joint_vars(true, false);
        fks = model.robot.joints2fk(joint_vars(1));

        % Save them in the accumulators
        jointsVsT = [jointsVsT; joint_vars(1)];
        fksVsT = [fksVsT; fks];
        times = [times; toc];
    end % Currently moving
end % pose loop

% XYZ position plot
plot(times, fksVsT(1,4,5));

