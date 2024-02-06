%% Choose poses for robot to move between
location = [200,  0,   100,  0;
            100,  200, 350, -90;
            50,  -70,  300, -45;
            200,  0,   100,  0];
move_time = 0; % seconds
plot_time = 3; % seconds of plotting time between moves
maxTrials = 3; %% Number of trials to perform

plot2d = false; % boolean to plot graphs
plot3d = false; % boolean to plot graphs
qplot = true; % boolean to plot graphs

%% Create model and robot

model = Model();

ik_pose = model.robot.task2ik(location(currTrial,:));
disp(ik_pose);
model.robot.set_joint_vars(ik_pose,3000); % nice, slow 3s move
disp(ik_pose);
pause(travelTime);

initPos = model.robot.read_joint_vars(true, false); % bool(pos,vel)
% Start timer
tic;

positionData = [toc, initPos(1,:)];
poseData = [toc, ik_pose(1,:)];

 %% 3D plot
 if plot3d 
        figure(2)
        title("3D Arm Live Model at Position ");
        xlabel("X (mm)");
        ylabel("Y (mm)");
        zlabel("Z (mm)");
        grid on
        set(gca,'fontsize',16)
        hold on
        [robotPlot, xQPlot, yQPlot, zQPlot] = model.new_arm_plots();
        hold off
 end

 %% Movement For Trajectory
for currTrial = 1:maxTrials
    ik_pose = model.robot.task2ik(location(currTrial+1,:));
    model.robot.set_joint_vars(ik_pose,move_time*1000);
    end_time = toc + plot_time;
    while toc <= end_time
        if plot3d 
            joint_vars = model.robot.read_joint_vars(true, false);
            fk = model.plot_arm(joint_vars(1, :), robotPlot, xQPlot, yQPlot, zQPlot);
            hold on
            scatter3(fk(1,4,4),fk(2,4,4),fk(3,4,4),100,"cyan");
            hold off      
        else 
            joint_vars = model.robot.read_joint_vars(true, false);
            positions = [toc, joint_vars(1,:)];
            positionData = cat(1,positionData,positions);
            pose = [toc, ik_pose(1,:)];
            poseData = cat(1,poseData,pose(1,:));
        end
        pause(0.001)
    end
end

%% 2d plot 
if plot2d 
    figure(1)  
    plot(positionData(2:end,1),positionData(2:end,2),'LineWidth',2);
    hold on
    plot(positionData(2:end,1),positionData(2:end,3),'LineWidth',2);
    plot(positionData(2:end,1),positionData(2:end,4),'LineWidth',2);
    plot(positionData(2:end,1),positionData(2:end,5),'LineWidth',2);
    hold off
    legend('Base angle', 'Shoulder Angle', 'Elbow Angle','Wrist Angle', 'Location', 'northwest')
    title('Joint Angles Trajectory of a Triangle From Inverse Kinematics')
    xlabel('Time (s)')
    ylabel('Pos (deg)')
    axis([0 max(positionData(:,1)) -90 90])
    set(gca,'fontsize',16)
end

%% q[1-4] plot
if qplot
    figure(3)
    plot(poseData(2:end,1),poseData(2:end,2),'LineWidth',2);
    hold on
    plot(poseData(2:end,1),poseData(2:end,3),'LineWidth',2);
    plot(poseData(2:end,1),poseData(2:end,4),'LineWidth',2);
    plot(poseData(2:end,1),poseData(2:end,5),'LineWidth',2);
    hold off
    legend('q1 (Base angle) ', 'q2 (Shoulder Angle) ', 'q3 (Elbow Angle)','q4 (Wrist Angle)', 'Location', 'northwest')
    title('q[1-4] Values From Inverse Kinematics of Triangle Trajectory')
    xlabel('Time (s)')
    ylabel('Pos (deg)')
    axis([0 max(positionData(:,1)) -90 90])
    set(gca,'fontsize',16)
end

%% Save the joint angles over time
save("no_interp_angles", "positionData");



