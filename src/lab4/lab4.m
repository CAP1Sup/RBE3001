clear all;
%% SETUP

model = Model();

location = [200,  0,   100,  0;
            100,  200, 350, -90;
            50,  -70,  300, -45;
            200,  0,   100,  0];

travelTime = 3; % seconds
maxTrials = 3; %% SET THIS NUMBER FOR EACH INDIVIDUAL TRIAL
currTrial = 1; % initalize trial start

plot3 = true; % IF FALSE YOU ARE DATA COLLECTING 
new_data = true;

if(new_data)
joint_vel = [0,0,0,0,0];
joint_pos = [0,0,0,0,0];
joint_value = [0,0,0,0,0; 0,0,0,0,0];
TsVel = [0,0,0,0,0,0];
time = [0];
EF=[0,0,0];
end

%% CODE

model.robot.set_joint_vars([0, 0, 0, 0], travelTime*1000);
pause(travelTime)
ik_pose = model.robot.task2ik(location(currTrial,:));
model.robot.set_joint_vars(ik_pose,travelTime*1000);
pause(travelTime);
currPos = model.robot.read_joint_vars(true , true);

for trial = currTrial:maxTrials
    coeffs = calc_quintic_t_coeff(model.robot,location(trial+1,:), travelTime);
    new_joint_value = model.robot.run_trajectory_pos_vel(coeffs, travelTime, true);
    
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

%% Perform calculations on the data
if(new_data)
    for i = 1:length(joint_value)
            if mod(i,2) == 0
            joint_vel = [joint_vel; joint_value(i,:)];
            else
            joint_pos = [joint_pos; joint_value(i,:)];
            time = [time; max(joint_pos(:,1))];
            end
    end % end for
    
time = time(3:end,1);               % gets rid of first row being 0
joint_vel = joint_vel(3:end,2:end); % gets rid of first row being 0
joint_pos = joint_pos(3:end,2:end); % gets rid of first row being 0


    for i = 1:length(time)
        TsVel = [TsVel;transpose(model.robot.vel2fdk(joint_pos(i,:),joint_vel(i,:)))];
        EF = [EF;model.robot.efCoord(joint_pos(i,:))];
    end % end for
    TsVel = TsVel(2:end,:);
    EF = EF(2:end,:);
end% end  if


%% 3D Live Plotting
if(plot3)
    figure(1)
    title("3D Arm Live Model at Position ")
    xlabel("X (mm)");
    ylabel("Y (mm)");
    zlabel("Z (mm)");
    grid on
    set(gca,'fontsize',16)
    [robotPlot, xQPlot, yQPlot, zQPlot] = model.new_arm_plots();
    hold on
    velQPlot = quiver3(0,0,0,0,0,0);
    hold off

    i = 1;
    tic;

    while(i <= length(time))   
        fks = model.plot_arm(joint_pos(i,:), robotPlot, xQPlot, yQPlot, zQPlot);
        set(velQPlot, "XData", fks(1,4,4), "YData", fks(2,4,4), "ZData", fks(3,4,4), "UData", TsVel(i,1), "VData", TsVel(i,2), "WData", TsVel(i,3));
        drawnow

        if (i == length(time))
            pause(0.1);
        else
            pause(time(i+1,1)-time(i,1));
        end
        i = i + 1;

    end % end while
end % end if plot


%% 2D Plotting
figure(2)
    plot(time(2:end), TsVel(2:end, 1));
    hold on
    plot(time(2:end), TsVel(2:end, 2));
    plot(time(2:end), TsVel(2:end, 3));
    hold off
    legend('EF X Velocity', 'EF Y Velocity', 'EF Z Velocity', 'Location', 'northwest')
    title('X,Y,Z Velocities of EF')
    xlabel('Time (s)')
    ylabel('Velocity (mm/s)')
    set(gca, 'fontsize', 18);
    

figure(3)
    plot(time(2:end), rad2deg(TsVel(2:end, 4)));
    hold on
    plot(time(2:end), rad2deg(TsVel(2:end, 5)));
    plot(time(2:end), rad2deg(TsVel(2:end, 6)));
    hold off
    legend('EF X Angular Velocity', 'EF Y Angular Velocity', 'EF Z Angular Velocity', 'Location', 'northwest')
    title('X,Y,Z Angular Velocities of EF')
    xlabel('Time (s)')
    ylabel('Velocity (deg/s)')
    set(gca, 'fontsize', 18);

mag = vecnorm(TsVel(2:end,1:3), 2, 2);
figure(4)
    plot(time(2:end), mag);
    legend('Magnitude of Linear Velocity', 'Location', 'northwest')
    title('EF Linear Velocity Magnitude')
    xlabel('Time (s)')
    ylabel('Velocity (mm/s)')
    set(gca, 'fontsize', 18);
    axis([0 max(time(:,1)) min(mag) max(mag)])

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