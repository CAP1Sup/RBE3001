%% Setup robot
travelTime = 3; % Defines the travel time
robot = Robot(); % Creates robot object
waypoint = [45,0,0,0]; % Give the angle for each position
%% Program
tic; % Start timer
initPos = robot.read_joint_vars(true , false);
initVel = robot.read_joint_vars(false , true);

readPos = [toc, initPos(1,:)];
readVel = [toc, initVel(1,:)];

robot.set_joint_vars(waypoint,(travelTime*1000));
 
while toc <= travelTime
     currPos = robot.read_joint_vars(true , false);
     P = [toc, currPos(1,:)];
     readPos = cat(1,readPos,P);

     currVel = robot.read_joint_vars(false , true);
     V = [toc, currVel(1,:)];
     readVel = cat(1,readVel,V);
     
end
     writematrix(readPos,'position.csv')
     writematrix(readVel,'velocity.csv')
     
%%Plotting
tiledlayout(2,2);
%Base Position Plotting
nexttile
     plot(readPos(:,1),readPos(:,2));
     title('Base Position Over Time (Base 0 to 45 deg)')
     xlabel('Time (s)')
     ylabel('Pos(deg)')
     axis([0 travelTime 0 45])
%Shoulder Position Plotting
nexttile
     plot(readPos(:,1),readPos(:,3));
     title('Shoulder Position Over Time (Base 0 to 45 deg)')
     xlabel('Time (s)')
     ylabel('Pos(deg)')
     axis([0 travelTime 0 45])
%Elbow Position Plotting
nexttile
     plot(readPos(:,1),readPos(:,4));
     title('Elbow Position Over Time (Base 0 to 45 deg)')
     xlabel('Time (s)')
     ylabel('Pos(deg)')
     axis([0 travelTime 0 45])
%Wrist Position Plotting
nexttile
     plot(readPos(:,1),readPos(:,5));
     title('Wrist Position Over Time (Base 0 to 45 deg)')
     xlabel('Time (s)')
     ylabel('Pos(deg)')
     axis([0 travelTime 0 45])
