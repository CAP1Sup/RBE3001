%% Setup robot
travelTime = 3; % Defines the travel time
%robot = Robot(); % Creates robot object
waypoint = [-80,-70,5,0]; % Give the angle for each position
home = [0,0,0,0]; % Home 0,0,0,0 position
deltaT= [0,0]; % initialize change in time matrix
maxTrials = 5; %% SET THIS NUMBER FOR EACH INDIVIDUAL TRIAL
currTrial = 1; % initalize trial start
%% Program
% robot.set_joint_vars(home,travelTime*1000);
% pause(travelTime);
% currPos = robot.read_joint_vars(true , false);
% positionData = robot.joints2fk(currPos(1,:));
% positionData = positionData(:,:,4);
% while currTrial <= maxTrials
%     robot.set_joint_vars(waypoint,travelTime*1000);
%     pause(travelTime);
% 
%     robot.set_joint_vars(home,travelTime*1000);
%     pause(travelTime + 0.5);
%     currPos = robot.read_joint_vars(true , false);
% 
%     tMatrix = robot.joints2fk(currPos(1,:));
%     tMatrix = tMatrix(:,:,4);
%     pause(0.5);
% disp(tMatrix);
%     positionData = cat(1,positionData,tMatrix);
% 
%     currTrial = currTrial + 1;
% 
% end
% 
% writematrix(positionData,'5trials_tMatrix.csv');% Always write current readings to position.csv
% figure(1)
% 
endeffactorPos = [positionData(5,4),positionData(6,4),positionData(7,4);
                  positionData(9,4),positionData(10,4),positionData(11,4);
                  positionData(13,4),positionData(14,4),positionData(15,4);
                  positionData(17,4),positionData(18,4),positionData(19,4);
                  positionData(21,4),positionData(22,4),positionData(23,4)];
%writematrix(endeffactorPos,'endEffactorPos.csv'); % uncomment if you want
C = [0 1 0; 1 0 0; 0.5 0.5 0.5; 0.6 0 1; 0 0 1];
X = [positionData(5,4) positionData(9,4) positionData(13,4) positionData(17,4) positionData(21,4)];
Y = [positionData(6,4) positionData(10,4) positionData(14,4) positionData(18,4) positionData(22,4)];
Z = [positionData(7,4) positionData(11,4) positionData(15,4) positionData(19,4) positionData(23,4)];
S = 75;

scatter3(positionData(5,4),positionData(6,4),positionData(7,4),450, 'b', 'DisplayName', 'trial1','lineWidth', 2)
xlabel('x pos (mm)')
ylabel('y pos (mm)')
zlabel('z pos (mm)')
title('5 Tip Positions With Respect To Base Frame')
set(gca,'fontsize',16)
legend('-DynamicLegend');
hold on
scatter3(positionData(9,4),positionData(10,4),positionData(11,4),375, 'g', 'DisplayName', 'trial2','lineWidth', 2)
xlabel('x pos (mm)')
ylabel('y pos (mm)')
zlabel('z pos (mm)')
set(gca,'fontsize',16)
legend('-DynamicLegend');
scatter3(positionData(13,4),positionData(14,4),positionData(15,4),250, 'r', 'DisplayName', 'trial3','lineWidth', 2)
xlabel('x pos (mm)')
ylabel('y pos (mm)')
zlabel('z pos (mm)')
set(gca,'fontsize',16)
legend('-DynamicLegend');
scatter3(positionData(17,4),positionData(18,4),positionData(19,4),175, 'm', 'DisplayName', 'trial4','lineWidth', 2)
xlabel('x pos (mm)')
ylabel('y pos (mm)')
zlabel('z pos (mm)')
set(gca,'fontsize',16)
legend('-DynamicLegend');
scatter3(positionData(21,4),positionData(22,4),positionData(23,4),50, 'k', 'DisplayName', 'trial5','lineWidth', 2)
xlabel('x pos (mm)')
ylabel('y pos (mm)')
zlabel('z pos (mm)')
set(gca,'fontsize',16)
legend('-DynamicLegend');
% scatter3(281.4,0,224.326,400,'diamond', 'color',[0.4940 0.1840 0.5560], 'DisplayName', 'ideal location', 'lineWidth', 2)
% xlabel('x pos (mm)')
% ylabel('y pos (mm)')
% zlabel('z pos (mm)')
% set(gca,'fontsize',16)
% legend('-DynamicLegend');
hold off




% xlim([282.2 282.5])
% ylim([-0.9 -0.4])
% zlim([211.2 211.5])
disp(endeffactorPos);

