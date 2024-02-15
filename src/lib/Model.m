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
        function [robotPlot, xQPlot, yQPlot, zQPlot] = new_arm_plots(~)
            hold on
            robotPlot = plot3(0, 0, 0, 1, 1, 1, "-r.");
            scaleFactor = 0.35;
            xQPlot = quiver3(0, 0, 0, 0, 0, 0, Color="r", AutoScaleFactor=scaleFactor);
            yQPlot = quiver3(0, 0, 0, 0, 0, 0, Color="g", AutoScaleFactor=scaleFactor);
            zQPlot = quiver3(0, 0, 0, 0, 0, 0, Color="b", AutoScaleFactor=scaleFactor);
            axis([-300, 300, -300, 300, -50, 400]);
            title("3D Plot of Robot with Reference Frames")
            xlabel("X Pos (mm)")
            ylabel("Y Pos (mm)")
            zlabel("Z Pos (mm)")
            hold off
        end % new_arm_plot

        % Takes a 1x4 array of joint values and the plot to update
        % Plots a stick model of the arm showing all frames, joints, and links
        function fk = plot_arm(self, joint_pos, robotPlot, xQPlot, yQPlot, zQPlot)
            fk = self.robot.joints2fk(joint_pos);
            xs = squeeze(fk(1,4,:));
            ys = squeeze(fk(2,4,:));
            zs = squeeze(fk(3,4,:));
            set(robotPlot, "XData", xs, "YData", ys, "ZData", zs);

            xhats = transpose(squeeze(fk(1:3,1,:)));
            yhats = transpose(squeeze(fk(1:3,2,:)));
            zhats = transpose(squeeze(fk(1:3,3,:)));
            set(xQPlot, "XData", xs, "YData", ys, "ZData", zs, "UData", xhats(:,1), "VData", xhats(:,2), "WData", xhats(:,3));
            set(yQPlot, "XData", xs, "YData", ys, "ZData", zs, "UData", yhats(:,1), "VData", yhats(:,2), "WData", yhats(:,3));
            set(zQPlot, "XData", xs, "YData", ys, "ZData", zs, "UData", zhats(:,1), "VData", zhats(:,2), "WData", zhats(:,3));
            drawnow
        end % plot_arm

    end % end methods
end % end class
