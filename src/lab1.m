%% Setup robot
displayTime = 5; % Defines the travel time
robot = Robot(); % Creates robot object
waypoint = [0,0,0,0];
%% Program
readPos = robot.read_joint_vars(true , false);
readVel = robot.read_joint_vars(false , true);
tic;

robot.set_joint_vars(waypoint)
 
while toc <= displayTime
     pos2 = robot.read_joint_vars(true , false);
     readPos = cat(1,readPos,pos2);
     vel2 = robot.read_joint_vars(false , true);
     readVel = cat(1,readVel,vel2);
end
     writematrix(readPos,'position.csv')
     writematrix(readVel,'velocity.csv')
     