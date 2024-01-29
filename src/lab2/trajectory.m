%% Setup robot
travelTime = 3; % Defines the travel time
%robot = Robot(); % Creates robot object
waypoint = [0, -25, -50,  90;
            0,  20,  20,  20;
            0,  0,  10,  0;
            0, -25, -50,  90]; % Give the angle for each position
home = [0,0,0,0]; % Home 0,0,0,0 position
deltaT= [0,0]; % initialize change in time matrix
maxTrials = 3; %% SET THIS NUMBER FOR EACH INDIVIDUAL TRIAL
currTrial = 1; % initalize trial start
%% Program
% robot.set_joint_vars(home,travelTime*1000);
% pause(travelTime);
% initPos = robot.read_joint_vars(true , false); % bool(pos,vel)
% 
% 
% robot.set_joint_vars(waypoint(currTrial, :),travelTime*1000);
% pause(travelTime);
% 
% tic; % Start timer
% timeVSdeltaT = [toc,toc];
% positionData = [toc, initPos(1,:)];
% 
% while currTrial <= maxTrials
% 
%       robot.set_joint_vars(waypoint(currTrial+1, :),travelTime*1000);
%       J = toc + travelTime;
%       while toc <= J
%             jointVars = robot.read_joint_vars(true , false);
% 
%             positions = [toc, jointVars(1,:)];
% 
%             positionData = cat(1,positionData,positions);
%       end
% 
%       currTrial = currTrial + 1;
% 
% end

% 
%writematrix(positionData,'triangle.csv');% Always write current readings to position.csv
% figure(1)


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




