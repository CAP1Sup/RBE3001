% (c) 2023 Robotics Engineering Department, WPI
% Skeleton Robot class for OpenManipulator-X Robot for RBE 3001

classdef Robot < OM_X_arm
    % Many properties are abstracted into OM_X_arm and DX_XM430_W350. classes
    % Hopefully, you should only need what's in this class to accomplish everything.
    % But feel free to poke around!
    properties
        mDim; % Stores the robot link dimensions (mm)
        mOtherDim; % Stores extraneous second link dimensions (mm)
        initialTranslation = [0, 0, 36.076]; % [x, y, z]
    end

    methods
        % Creates constants and connects via serial
        % Super class constructor called implicitly
        % Add startup functionality here
        function self = Robot()
            % Change robot to position mode with torque enabled by default
            % Feel free to change this as desired
            self.writeMode('p');
            self.writeMotorState(true);

            % Set the robot to move between positions with a 5 second profile
            % change here or call writeTime in scripts to change
            self.writeTime(2);

            % Robot Dimensions
            self.mDim = [96.326, 130.23, 124, 133.4]; % (mm)
            self.mOtherDim = [128, 24]; % (mm)
        end

        % Sends the joints to the desired angles
        % goals [1x4 double] - angles (degrees) for each of the joints to go to
        function writeJoints(self, goals)
            goals = mod(round(goals .* DX_XM430_W350.TICKS_PER_DEG + DX_XM430_W350.TICK_POS_OFFSET), DX_XM430_W350.TICKS_PER_ROT);
            self.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.GOAL_POSITION, goals);
        end

        % Gets the joint's goal positions
        function goals = getJointPosGoals(self)
            goals = (self.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.GOAL_POSITION) - DX_XM430_W350.TICK_POS_OFFSET) ./ DX_XM430_W350.TICKS_PER_DEG;
        end

        % Creates a time based profile (trapezoidal) based on the desired times
        % This will cause writePosition to take the desired number of
        % seconds to reach the setpoint. Set time to 0 to disable this profile (be careful).
        % time [double] - total profile time in s. If 0, the profile will be disabled (be extra careful).
        % acc_time [double] - the total acceleration time (for ramp up and ramp down individually, not combined)
        % acc_time is an optional parameter. It defaults to time/3.

        function writeTime(self, time, acc_time)

            if (~exist("acc_time", "var"))
                acc_time = time / 3;
            end

            time_ms = time * DX_XM430_W350.MS_PER_S;
            acc_time_ms = acc_time * DX_XM430_W350.MS_PER_S;

            % disp("time")
            % disp(time_ms)
            % disp("acc time")
            % disp(acc_time_ms)

            self.bulkReadWrite(DX_XM430_W350.PROF_ACC_LEN, DX_XM430_W350.PROF_ACC, acc_time_ms);
            self.bulkReadWrite(DX_XM430_W350.PROF_VEL_LEN, DX_XM430_W350.PROF_VEL, time_ms);
        end

        % Sets the gripper to be open or closed
        % Feel free to change values for open and closed positions as desired (they are in degrees)
        % open [boolean] - true to set the gripper to open, false to close
        function writeGripper(self, open)

            if open
                self.gripper.writePosition(-35);
            else
                self.gripper.writePosition(55);
            end

        end

        % Sets position holding for the joints on or off
        % enable [boolean] - true to enable torque to hold last set position for all joints, false to disable
        function writeMotorState(self, enable)
            self.bulkReadWrite(DX_XM430_W350.TORQUE_ENABLE_LEN, DX_XM430_W350.TORQUE_ENABLE, enable);
        end

        % Supplies the joints with the desired currents
        % currents [1x4 double] - currents (mA) for each of the joints to be supplied
        function writeCurrents(self, currents)
            currentInTicks = round(currents .* DX_XM430_W350.TICKS_PER_mA);
            self.bulkReadWrite(DX_XM430_W350.CURR_LEN, DX_XM430_W350.GOAL_CURRENT, currentInTicks);
        end

        % Change the operating mode for all joints:
        % https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#operating-mode11
        % mode [string] - new operating mode for all joints
        % "current": Current Control Mode (writeCurrent)
        % "velocity": Velocity Control Mode (writeVelocity)
        % "position": Position Control Mode (writePosition)
        % Other provided but not relevant/useful modes:
        % "ext position": Extended Position Control Mode
        % "curr position": Current-based Position Control Mode
        % "pwm voltage": PWM Control Mode
        function writeMode(self, mode)

            switch mode
                case {'current', 'c'}
                    writeMode = DX_XM430_W350.CURR_CNTR_MD;
                case {'velocity', 'v'}
                    writeMode = DX_XM430_W350.VEL_CNTR_MD;
                case {'position', 'p'}
                    writeMode = DX_XM430_W350.POS_CNTR_MD;
                case {'ext position', 'ep'} % Not useful normally
                    writeMode = DX_XM430_W350.EXT_POS_CNTR_MD;
                case {'curr position', 'cp'} % Not useful normally
                    writeMode = DX_XM430_W350.CURR_POS_CNTR_MD;
                case {'pwm voltage', 'pwm'} % Not useful normally
                    writeMode = DX_XM430_W350.PWM_CNTR_MD;
                otherwise
                    error("setOperatingMode input cannot be '%s'. See implementation in DX_XM430_W350. class.", mode)
            end

            lastVelTimes = self.bulkReadWrite(DX_XM430_W350.PROF_VEL_LEN, DX_XM430_W350.PROF_VEL);
            lastAccTimes = self.bulkReadWrite(DX_XM430_W350.PROF_ACC_LEN, DX_XM430_W350.PROF_ACC);

            self.writeMotorState(false);
            self.bulkReadWrite(DX_XM430_W350.OPR_MODE_LEN, DX_XM430_W350.OPR_MODE, writeMode);
            self.writeTime(lastVelTimes(1) / 1000, lastAccTimes(1) / 1000);
            self.writeMotorState(true);
        end

        % Gets the current joint positions, velocities, and currents
        % readings [3x4 double] - The joints' positions, velocities,
        % and efforts (deg, deg/s, mA)
        function readings = getJointsReadings(self)
            readings = zeros(3, 4);

            readings(1, :) = (self.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.CURR_POSITION) - DX_XM430_W350.TICK_POS_OFFSET) ./ DX_XM430_W350.TICKS_PER_DEG;
            readings(2, :) = self.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.CURR_VELOCITY) ./ DX_XM430_W350.TICKS_PER_ANGVEL;
            readings(3, :) = self.bulkReadWrite(DX_XM430_W350.CURR_LEN, DX_XM430_W350.CURR_CURRENT) ./ DX_XM430_W350.TICKS_PER_mA;
        end

        % Sends the joints at the desired velocites
        % vels [1x4 double] - angular velocites (deg/s) for each of the joints to go at
        function writeVelocities(self, vels)
            vels = round(vels .* DX_XM430_W350.TICKS_PER_ANGVEL);
            self.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.GOAL_VELOCITY, vels);
        end

        % Returns a 2x4 array with joint positions and velocities
        % Position and velocities default to zero
        % GETPOS and GETVEL control if their data is included
        function readings = read_joint_vars(self, GETPOS, GETVEL)
            readings = zeros(2, 4);

            if GETPOS
                readings(1, :) = (self.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.CURR_POSITION) - DX_XM430_W350.TICK_POS_OFFSET) ./ DX_XM430_W350.TICKS_PER_DEG;
            end

            if GETVEL
                readings(2, :) = self.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.CURR_VELOCITY) ./ DX_XM430_W350.TICKS_PER_ANGVEL;
            end

        end

        % Takes a 1x4 array of joint variables (in degrees) to be sent to the actuators
        % Optional parameter that specifies the desired travel time (in milliseconds)
        function set_joint_vars(varargin)
            % Set the self variable for better readability
            self = varargin{1};

            % Check the number of input arguments
            if length(varargin) ~= 2 && length(varargin) ~= 3
                error("Incorrect number of input arguments. Expected 1 or 2.");
            end

            % Extract joint variables
            jointVars = varargin{2};

            % Validate jointVars array size
            if size(jointVars, 1) ~= 1 || size(jointVars, 2) ~= 4

                error("Joint variables must be a 1x4 array.");
            end

            % Check for optional travel time argument
            if length(varargin) == 3
                self.writeTime(varargin{3} / 1000);
            end

            % Set the joint angles
            self.writeJoints(jointVars);
        end

        % Gets a DH table using the current joint positions
        % Takes in a 1x4 vector of joint angles in deg
        function dhTable = getDHTable(~, joint_pos)
            % Initialize the generalized DH table
            dhTable = [0, 60.25, 0, -pi / 2;
                       -atan2(128, 24), 0 sqrt(24 ^ 2 + 128 ^ 2), 0;
                       atan2(128, 24), 0, 124, 0;
                       0, 0, 133.4, 0];

            % Add the joint angles
            dhTable(:, 1) = dhTable(:, 1) + deg2rad(transpose(joint_pos));
        end % getDHTable

        % Takes a 1x4 array of DH parameters in form: [theta, d, a, alpha]
        % Returns a 4x4 frame transformational matrix
        function dhMat = dh2mat(~, DHrow)
            theta = DHrow(1, 1);
            d = DHrow(1, 2);
            a = DHrow(1, 3);
            alpha = DHrow(1, 4);

            dhMat = [cos(theta) -sin(theta) * cos(alpha) sin(theta) * sin(alpha) a * cos(theta);
                     sin(theta) cos(theta) * cos(alpha) -cos(theta) * sin(alpha) a * sin(theta);
                     0 sin(alpha) cos(alpha) d;
                     0 0 0 1];
        end % dh2mat

        % Takes in an nx4 array corresponding to the n rows of the full DH parameter table
        % Each joint gets a 4x4 homogenous matrix that gives the global orientation and position of the joint
        % Outputs a (4x4)xn array where “n” is the number of joints
        function dhfk_mats = dh2fk(self, DHparams)
            % Number of joints
            n = size(DHparams, 1);

            % Initialize array to hold transformation matrices
            dhfk_mats = zeros(4, 4, n);

            % Initialize the transformation as an identity matrix
            dhfk_prev = eye(4);
            dhfk_prev(1:3, 4) = self.initialTranslation;

            for joint_index = 1:n
                % Compute the transformation matrix for the current joint
                dhfk_current = self.dh2mat(DHparams(joint_index, :));

                % Accumulate the transformation
                dhfk_prev = dhfk_prev * dhfk_current;

                % Store the transformation in the output array
                dhfk_mats(:, :, joint_index) = dhfk_prev;
            end

        end % dh2fk

        % Takes a 1x4 vector of joint angles
        % Outputs the full transformations from the reference frame
        function fk = joints2fk(self, joint_pos)
            fk = self.dh2fk(self.getDHTable(joint_pos));
        end % joints2fk

        % Takes a 1x4 vector of x,y,z position in mm and orientation angle,
        % alpha in degrees
        % Ouputs a 1x4 vector of joint angles
        function jointParam = task2ik(~, pos_arr)
            % Define constants and value naming
            x = pos_arr(1);
            y = pos_arr(2);
            z = pos_arr(3);
            alpha = deg2rad(pos_arr(4));
            L1x = 24;
            L1y = 128;
            L2 = 124;
            L3 = 133.4;

            % Compute side lengths of the triangle
            planar_x = sqrt(x ^ 2 + y ^ 2) - L3 * cos(alpha);
            planar_y = z + L3 * sin(alpha) - 96.326;
            a = sqrt(planar_x ^ 2 + planar_y ^ 2);
            L1 = sqrt(L1x ^ 2 + L1y ^ 2); % L1

            % Calculate the angles of the triangle
            if abs((L2 ^ 2 + L1 ^ 2 - a ^ 2) / (2 * L2 * L1)) > 1
                error("Cannot be reached, elbow joint angle does not exist");
            else
                A = acos((L2 ^ 2 + L1 ^ 2 - a ^ 2) / (2 * L2 * L1));
            end

            if abs((a ^ 2 + L1 ^ 2 - L2 ^ 2) / (2 * a * L1)) > 1
                error("Cannot be reached, shoulder joint angle does not exist");
            else
                B = acos((a ^ 2 + L1 ^ 2 - L2 ^ 2) / (2 * a * L1));
            end

            % Calculate the angle of the J1->J3 vector and sholder offset
            beta = atan2(planar_y, planar_x);
            shoulder_offset = atan2(L1x, L1y);

            % Compute the location of the left-right rotation
            if x == 0 && y == 0
                q1 = 0;
            else
                q1 = atan2(y, x);
            end

            % Calculate the joint angles
            q2 = pi / 2 - B - beta - shoulder_offset;
            q3 = pi / 2 - A + shoulder_offset;
            q4 = alpha - q3 - q2;

            % Quick sanity check on the wrist angle
            if abs(q4) > pi / 2 + deg2rad(10)
                error("Cannot be reached, wrist joint angle does not exist");
            end

            jointParam = rad2deg([q1, q2, q3, q4]);
        end % task2ik

        % Takes 4x4 matrix of trajectory coefficients, with joints or task
        % space dimensions [x,y,z,alpha] as rows and coefficients as
        % columns, and the desired movement duration in seconds. Specify
        % "true" as the third argument if the trajectories are in task
        % space instead of joint space.
        % Returns joint angle data entries of format: [time, q1, q2, q3, q4]
        function joint_pos_data = run_trajectory(varargin)
            % Validate argument count
            if length(varargin) ~= 3 && length(varargin) ~= 4
                error("Invalid number of arguments in run_trajectory");
            end

            % Set variables for easier readability
            self = varargin{1};
            traj_coef = varargin{2};
            move_dur = varargin{3};

            tic;

            while toc <= move_dur
                time = toc;

                % Decide if we're in task space
                if length(varargin) == 4
                    in_task_space = varargin{4};
                else
                    in_task_space = false;
                end

                if (in_task_space)
                    x = polyval(flip(traj_coef(1, :), 2), time);
                    y = polyval(flip(traj_coef(2, :), 2), time);
                    z = polyval(flip(traj_coef(3, :), 2), time);
                    alpha = polyval(flip(traj_coef(4, :), 2), time);
                    joint_vals = self.task2ik([x, y, z, alpha]);
                else % in joint space
                    q1 = polyval(flip(traj_coef(1, :), 2), time);
                    q2 = polyval(flip(traj_coef(2, :), 2), time);
                    q3 = polyval(flip(traj_coef(3, :), 2), time);
                    q4 = polyval(flip(traj_coef(4, :), 2), time);
                    joint_vals = [q1, q2, q3, q4];
                end

                self.set_joint_vars(joint_vals, 0);
                pause(0.01); % pause 0.01s so joints can catch up
                joint_pos = self.read_joint_vars(true, false);

                if exist("joint_pos_data", "var")
                    joint_pos_data = [joint_pos_data; [time, joint_pos(1, :)]];
                else
                    joint_pos_data = [time, joint_pos(1, :)];
                end % exists "joint_pos_data"

            end % toc <= move_dur

        end % run_trajectory

        % Takes in in a 1x4 matrix of current joint angles (in deg)
        % Produces corresponding numeric 6x4 Jacobian matrix
        % Position part of Jacobian was generated by calc_sym_jacobian.m
        % script in lab4 folder
        function jacobian = get_jacobian(~, q)
            % Jacobian table uses radians, so we have to convert
            q = deg2rad(q);

            % Massive Jacobian table... :O
            jacobian = [-0.2 * sin(q(1)) * (651.15 * cos(q(2) - 1.3854) + 667.0 * cos(q(2) + q(3) + q(4)) + 620.0 * cos(q(2) + q(3))), ...
                            -0.2 * cos(q(1)) * (651.15 * sin(q(2) - 1.3854) + 667.0 * sin(q(2) + q(3) + q(4)) + 620.0 * sin(q(2) + q(3))), ...
                            -0.2 * cos(q(1)) * (667.0 * sin(q(2) + q(3) + q(4)) + 620.0 * sin(q(2) + q(3))), ...
                            -133.4 * cos(q(1)) * sin(q(2) + q(3) + q(4));
                        0.2 * cos(q(1)) * (651.15 * cos(q(2) - 1.3854) + 667.0 * cos(q(2) + q(3) + q(4)) + 620.0 * cos(q(2) + q(3))), ...
                            -0.2 * sin(q(1)) * (651.15 * sin(q(2) - 1.3854) + 667.0 * sin(q(2) + q(3) + q(4)) + 620.0 * sin(q(2) + q(3))), ...
                            -0.2 * sin(q(1)) * (667.0 * sin(q(2) + q(3) + q(4)) + 620.0 * sin(q(2) + q(3))), ...
                            -133.4 * sin(q(1)) * sin(q(2) + q(3) + q(4));
                        0, ...
                            -130.23 * cos(q(2) - 1.3854) - 133.4 * cos(q(2) + q(3) + q(4)) - 124.0 * cos(q(2) + q(3)), ...
                            -133.4 * cos(q(2) + q(3) + q(4)) - 124.0 * cos(q(2) + q(3)), ...
                            -133.4 * cos(q(2) + q(3) + q(4));

            % Orientation Jacobian
                        0, -sin(q(1)), -sin(q(1)), -sin(q(1));
                        0, cos(q(1)), cos(q(1)), cos(q(1));
                        1, 0, 0, 0];
        end % get_jacobian

        % Takes a 1x4 vector of current joint angles and a 1x4 vector of
        % instantaneous joint velocites
        % Returns a 6x1 vector of task-space linear & angular velocities
        function TSvel = vel2fdk(self, curr_joint_ang, inst_joint_vel)
            jacobian = self.get_jacobian(curr_joint_ang);
            TSvel = jacobian * transpose(inst_joint_vel);
        end % end vel2fdk

        % Takes a 1x3 vector of target joint position (x, y, z)
        % Returns a 1x4 vector of target joint angles
        function final_joint_pos = Newton_Raphson_IK(self, target_task_pos)
            % Gets the 1x4 array of the joint positions
            % Used to "seed" the integration
            curr_joint_var = self.read_joint_vars(true, false);
            curr_joint_pos = curr_joint_var(1, :);

            % Performs a fk with current joint values and gets the 1x3
            % array of position
            fks = joints2fk(curr_joint_pos);
            curr_task_pos = transpose(fks(1:3, 4, 4));

            % Algorithm runs until the desired error tolerance is reached
            while norm(target_task_pos - curr_task_pos) > 1e-3
                % Gets the Jacobian
                jacobian = self.get_jacobian(curr_joint_pos);

                % Performs a fk with current joint values and gets the 1x3
                % array of position
                fks = joints2fk(curr_joint_pos);
                curr_task_pos = transpose(fks(1:3, 4, 4));

                % Calculates the change in joint positions
                % Gets the first 3x4 Jacobian (only x, y, z)
                deltaQ = pinv(jacobian(3, :)) * (target_task_pos - curr_task_pos);

                % Updates the joint positions
                curr_joint_pos = curr_joint_pos + deltaQ;
            end

            % Read the joint angles after it reached the target position
            final_joint_pos = self.read_joint_vars(true, false);
            final_joint_pos = final_joint_pos(1, :);
        end % Newton_Raphson_IK

    end % end methods

end % end class
