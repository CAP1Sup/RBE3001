%% Setup robot
travelTime = 3; % Defines the travel time
robot = Robot(); % Creates robot object
waypoint = [0, -25, -50,  90;
            0,  20,  20,  20;
            0,  0,  10,  0;
            0, -25, -50,  90]; % Give the angle for each position
home = [0,0,0,0]; % Home 0,0,0,0 position
deltaT= [0,0]; % initialize change in time matrix
maxTrials = 3; %% SET THIS NUMBER FOR EACH INDIVIDUAL TRIAL
currTrial = 1; % initalize trial start
%% Program
robot.set_joint_vars(home,travelTime*1000);
pause(travelTime);
initPos = robot.read_joint_vars(true , false); % bool(pos,vel)


robot.set_joint_vars(waypoint(currTrial, :),travelTime*1000);
pause(travelTime);

tic; % Start timer
timeVSdeltaT = [toc,toc];
positionData = [toc, initPos(1,:)];

currPos = robot.read_joint_vars(true , false);

            x_tMatrix = robot.joints2fk(currPos(1,:));
            x_tMatrix = x_tMatrix(1,4,4);
            z_tMatrix = robot.joints2fk(currPos(1,:));
            z_tMatrix = z_tMatrix(3,4,4);
            y_tMatrix = robot.joints2fk(currPos(1,:));
            y_tMatrix = y_tMatrix(2,4,4);

efXZ_Position = [toc,x_tMatrix,z_tMatrix,y_tMatrix];

while currTrial <= maxTrials

      robot.set_joint_vars(waypoint(currTrial+1, :),travelTime*1000);
      J = toc + travelTime;
      while toc <= J
            jointVars = robot.read_joint_vars(true , false);

            currPos = robot.read_joint_vars(true , false);

            x_tMatrix = robot.joints2fk(currPos(1,:));
            x_tMatrix = x_tMatrix(1,4,4);
            z_tMatrix = robot.joints2fk(currPos(1,:));
            z_tMatrix = z_tMatrix(3,4,4);
            y_tMatrix = robot.joints2fk(currPos(1,:));
            y_tMatrix = y_tMatrix(2,4,4);

            efXZCurrentPosition = [toc,x_tMatrix,z_tMatrix,y_tMatrix];

            positions = [toc, jointVars(1,:)];

            positionData = cat(1,positionData,positions);

            efXZ_Position = cat(1,efXZ_Position,efXZCurrentPosition);

      end

      currTrial = currTrial + 1;

end
writematrix(efXZ_Position,'efXZPosition.csv');
% 
%writematrix(positionData,'triangle.csv');% Always write current readings to position.csv
 figure(1) % 


plot(positionData(2:end,1),positionData(2:end,2),'LineWidth',2);
    hold on
    plot(positionData(2:end,1),positionData(2:end,3),'LineWidth',2);
    plot(positionData(2:end,1),positionData(2:end,4),'LineWidth',2);
    plot(positionData(2:end,1),positionData(2:end,5),'LineWidth',2);

    hold off
    legend('Base angle', 'Shoulder Angle', 'Elbow Angle','Wrist Angle', 'Location', 'northwest')
    title('Joint Angles Trajectory of a Triangle')
    xlabel('Time (s)')
    ylabel('Pos (deg)')
    axis([0 max(positionData(:,1)) -90 90])
    set(gca,'fontsize',16)

figure(2) % x and z end effactor position vs time 

plot(efXZ_Position(2:end,1),efXZ_Position(2:end,2));% x
hold on
plot(efXZ_Position(2:end,1),efXZ_Position(2:end,3));% z
hold off
title('End Effector Position in X and Z Axes vs Time')
legend('X Position', 'Z Position', 'Location', 'north')
xlabel('Time (s)')
ylabel('Pos (mm)')
axis([0 max(efXZ_Position(:,1)) 0 310])
set(gca,'fontsize',16)



figure(3)

[Xmin,Z] = min(efXZ_Position(:,2));
[Xmax,Z2] = max(efXZ_Position(:,2));
[Zmin,X1] = min(efXZ_Position(:,3));

plot(efXZ_Position(2:end,2),efXZ_Position(2:end,3), 'DisplayName', 'trajectory','LineWidth',1);
hold on
plot(Xmin,efXZ_Position(Z,3), 'o', 'DisplayName', 'vertice 1','LineWidth',3);
plot(Xmax,efXZ_Position(Z2,3), 'o', 'DisplayName', 'vertice 2','LineWidth',3);
plot(efXZ_Position(X1,2),Zmin, 'o', 'DisplayName', 'vertice 3','LineWidth',3);
hold off 
legend('-DynamicLegend');
title('Triangle Trajectory')
xlabel('X pos (mm)')
ylabel('Z Pos (mm)')
axis([125 300 0 310])
set(gca,'fontsize',16)


figure(4)

[Ymin,Z] = min(efXZ_Position(:,4));
[Ymax,Z2] = max(efXZ_Position(:,4));

plot(efXZ_Position(2:end,2),efXZ_Position(2:end,4), 'DisplayName', 'Y displacment','LineWidth',1);
hold on
plot(efXZ_Position(Z,2),Ymin, 'o', 'DisplayName', 'Minimum Y Displacement','LineWidth',3);
plot(efXZ_Position(Z2,2),Ymax, 'o', 'DisplayName', 'Maxmimum Y Displacement','LineWidth',3);
hold off 
legend('-DynamicLegend');
title('Triangle Trajectory Y Displacement')
xlabel('X pos (mm)')
ylabel('Y Pos (mm)')
axis([125 300 -0.5 0.5])
set(gca,'fontsize',16)


