classdef Model
    % Used to simulate the robot arm
    properties
        robot; % Stores an instance of the Robot class for interaction and kinematics
    end

    methods
        % Creates instance of Robot class
        function self = Model()
            self.robot = Robot();
        end

        % Creates a new plot for displaying the arm
        function robotPlot = new_arm_plot(~)
            robotPlot = plot3(0, 0, 0, "-r.");
            axis([-300, 300, -300, 300, 0, 400]);
        end % new_arm_plot

        % Takes a 1x4 array of joint values and the plot to update
        % Plots a stick model of the arm showing all frames, joints, and links
        function plot_arm(self, joint_pos, robotPlot)
            fk = self.robot.joints2fk(joint_pos);
            set(robotPlot, "XData", fk(1,4,:), "YData", fk(2,4,:), "ZData", fk(3,4,:));
            drawnow
        end % plot_arm

    end % end methods
end % end class 
