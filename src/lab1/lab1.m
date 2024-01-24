%% Setup robot
travelTime = 3; % Defines the travel time
robot = Robot(); % Creates robot object
waypoint = [-80,-70,5,0]; % Give the angle for each position
%waypoint = [45,0,0,0];
home = [0,0,0,0]; % Home 0,0,0,0 position
deltaT= [0,0]; % initialize change in time matrix
trialNum = 0; %% SET THIS NUMBER FOR EACH INDIVIDUAL TRIAL
%% Program
robot.set_joint_vars(home);
pause(travelTime);
tic; % Start timer
timeVSdeltaT = [toc,toc];
initPos = robot.read_joint_vars(true , false); % bool(pos,vel)

positionData = [toc, initPos(1,:)];

robot.set_joint_vars(waypoint,(travelTime*1000));

timeVSdeltaT = [toc, abs(timeVSdeltaT(1)-toc)*1000];% toc (seconds) deltaToc (milliseconds)

while toc <= travelTime
     
     jointVars = robot.read_joint_vars(true , false);
     positions = [toc, jointVars(1,:)];
     positionData = cat(1,positionData,positions);

     timeVSdeltaT = [toc, abs(timeVSdeltaT(1)-toc)*1000];% toc (seconds) deltaToc (milliseconds)
     deltaT = cat(1,deltaT,timeVSdeltaT);
end
     writematrix(positionData,'45DegPosition.csv');% Always write current readings to position.csv
     writematrix(deltaT(2:end,:),'deltaT.csv'); % 1st line is 0,0 so ignore this line and start from 2nd

%% Trial Position Writing To CSV
if trialNum == 1
    writematrix(positionData, 'position1.csv');
end
if trialNum == 2
    writematrix(positionData, 'position2.csv');
end
if trialNum == 3
    writematrix(positionData, 'position3.csv');
end

    
     trial1 = readmatrix('position1.csv');
     trial2 = readmatrix('position2.csv');
     trial3 = readmatrix('position3.csv');



%% Plotting

figure(1);
tiledlayout(2,2);
if waypoint(2) == 0 && waypoint(3) == 0 & waypoint(4) ==0
% Base Position Plotting
nexttile
     plot(positionData(:,1),positionData(:,2));
     title("Base Position Over Time (Base 0 to " + waypoint(1) + " deg)")
     xlabel('Time (s)')
     ylabel('Pos(deg)')
     axis([0 travelTime 0 45])
     set(gca,'fontsize',16)
% Shoulder Position Plotting
nexttile
     plot(positionData(:,1),positionData(:,3));
     title("Shoulder Position Over Time (Base 0 to " + waypoint(1) + " deg)")
     xlabel('Time (s)')
     ylabel('Pos(deg)')
     axis([0 travelTime 0 45])
     set(gca,'fontsize',16)
% Elbow Position Plotting
nexttile
     plot(positionData(:,1),positionData(:,4));
     title("Elbow Position Over Time (Base 0 to " + waypoint(1) + " deg)")
     xlabel('Time (s)')
     ylabel('Pos(deg)')
     axis([0 travelTime 0 45])
     set(gca,'fontsize',16)
% Wrist Position Plotting
nexttile
     plot(positionData(:,1),positionData(:,5));
     title("Wrist Position Over Time (Base 0 to " + waypoint(1) + " deg)")
     xlabel('Time (s)')
     ylabel('Pos(deg)')
     axis([0 travelTime 0 45])
     set(gca,'fontsize',16)
end

figure(2);
% Histogram
tiledlayout(1,2)
nexttile
     histogram(deltaT(2:end,2));
     title('Incremental Timesteps Between Readings')
     xlabel('Time (ms)')
     ylabel('count')
set(gca,'fontsize',16)
% 3Layer Plot
nexttile
    plot(trial1(:,1),trial1(:,2));
    hold on
    plot(trial2(:,1),trial2(:,2));
    plot(trial3(:,1),trial3(:,2));
    hold off
    legend('trial1', 'trial2', 'trial3', 'Location', 'northwest')
    title('Base Position Over Time Over 3 Trials (0 to 78 deg)')
    xlabel('Time (s)')
    ylabel('Pos(deg)')
    axis([0 travelTime 0 80])
    set(gca,'fontsize',16)

figure(3);
tiledlayout(2,2)
% Base Position Plotting
nexttile
     plot(positionData(:,1),positionData(:,2));
     title("Base Position Over Time (Base 0 to " + waypoint(1) + " deg)")
     xlabel('Time (s)')
     ylabel('Pos(deg)')
     axis([0 travelTime min(positionData(:,2)) max(positionData(:,2))])
     set(gca,'fontsize',16)

% Shoulder Position Plotting
nexttile
     plot(positionData(:,1),positionData(:,3));
     title("Shoulder Position Over Time (Shoulder 0 to " + waypoint(2) + " deg)")
     xlabel('Time (s)')
     ylabel('Pos(deg)')
     axis([0 travelTime min(positionData(:,3)) max(positionData(:,3))])
     set(gca,'fontsize',16)

% Elbow Position Plotting
nexttile
     plot(positionData(:,1),positionData(:,4));
     title("Elbow Position Over Time (Elbow 0 to " + waypoint(3) + " deg)")
     xlabel('Time (s)')
     ylabel('Pos(deg)')
     axis([0 travelTime min(positionData(:,4)) max(positionData(:,4))])
     set(gca,'fontsize',16)

% Wrist Position Plotting
nexttile
     plot(positionData(:,1),positionData(:,5));
     title("Wrist Position Over Time (Wrist 0 to " + waypoint(4) + " deg)")
     xlabel('Time (s)')
     ylabel('Pos(deg)')
     axis([0 travelTime min(positionData(:,5)) max(positionData(:,5))])
set(gca,'fontsize',16)

