% (c) 2023 Robotics Engineering Department, WPI
% Class for OpenManipulator-X arm
% Used to abstract serial connection and read/write methods from Robot class
% This class/file should not need to be modified in any way for RBE 3001

classdef Dummy_OM_X_arm < handle
    properties
        % Values to keep track of while simulating
        pos_goals = [0,0,0,0];
        vel_goals = [0,0,0,0];
    end

    methods
        % Constructor to create constants and connect via serial
        % Fun fact, only the motor IDs are needed for bulk read/write,
        % so no instances of the DX_XM430_W350 class are required for the joints.
        % We still provide them (see self.motors) because we found them useful for
        % controlling the motors individually on occasion.
        function self = Dummy_OM_X_arm()
        end

        % Reads or Writes the message of length n from/to the desired address for all joints
        % result [integer 1x4] - the result of a bulk read, empty if bulk write
        % n [integer] - The size in bytes of the message 
        % (1 for most settings, 2 for current, 4 for velocity/position)
        % addr [integer] - address of control table index to read from or write to
        % msgs [integer 1x4] - the messages (in bytes) to send to each joint, respectively
        % msgs is optional. Do not provide if trying to read.
        % msgs can also be an integer, where the same message will be sent to all four joints.
        function result = bulkReadWrite(self, n, addr, msgs)

            if (n==DX_XM430_W350.POS_LEN && addr==DX_XM430_W350.GOAL_POSITION)
                self.pos_goals = msgs;
                self.vel_goals = [0,0,0,0];
            end % Set desired position command

            if (n==DX_XM430_W350.VEL_LEN && addr==DX_XM430_W350.GOAL_VELOCITY)
                self.vel_goals = msgs;
                self.pos_goals = [0,0,0,0];
            end % Set desired velocity command

            % Decide what to return
            if (n==DX_XM430_W350.POS_LEN && addr==DX_XM430_W350.CURR_POSITION)
                % Get current position command
                result = self.pos_goals;
            elseif (n==DX_XM430_W350.VEL_LEN && addr==DX_XM430_W350.CURR_VELOCITY)
                % Get current velocity command 
                result = self.vel_goals;
            else
                result = [0,0,0,0];
            end
        end
    end % end methods
end % end class