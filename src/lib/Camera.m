classdef Camera < handle
    % CAMERA Example Camera class for RBE 3001 Lab 5
    %   You can add your image processing in this camera class,
    %   as well as any other functions related to the camera.
    
    properties        
        % Properties
        params;     % Camera Parameters
        cam;        % Webcam Object
        cam_pose;   % Camera Pose (transformation matrix)
        cam_IS;     % Camera Intrinsics
        cam_R;      % Camera Rotation Matrix
        cam_T;      % Camera Translation Vector
        cam_TForm   % Camera Rigid 3D TForm
    end
    
    methods
        function self = Camera()
            % CAMERA Construct an instance of this class
            % make sure that the webcam can see the whole checkerboard by
            % running webcam(2).preview in the Command Window
            self.cam = webcam(2); % Get camera object
            self.params = self.calibrate(); % Run Calibration Function
            [self.cam_IS, self.cam_pose] = self.calculateCameraPos();
        end

        function tForm = getTForm(self)
            tForm = self.cam_TForm;
        end

        function cam_pose = getCameraPose(self)
            cam_pose = self.cam_pose;
        end

        function cam_IS = getCameraIntrinsics(self)
            cam_IS = self.cam_IS;
        end

        function cam_R = getRotationMatrix(self)
            cam_R = self.cam_R;
        end

        function cam_T = getTranslationVector(self)
            cam_T = self.cam_T;
        end

        function shutdown(self)
            % SHUTDOWN shutdown script which clears camera variable
            clear self.cam;
        end
      
        function params = calibrate(self)
            % CALIBRATE Calibration function
            % This function will run the camera calibration, save the camera parameters,
            % and check to make sure calibration worked as expected
            % The calibrate function will ask if you are ready. To calibrate, you must press
            % any key, then the system will confirm if the calibration is successful

            % NOTE: This uses the camcalib.m file for camera calibration. If you have placed
            % your camera calibration script elsewhere, you will need to change the command below

            params = 0;
            try
                disp("Clear surface of any items, then press any key to continue");
                pause;
                disp("Calibrating");
                camcalib; % Change this if you are using a different calibration script
                params = cameraParams;
                disp("Camera calibration complete!");
            catch exception
                msg = getReport(exception);
                disp(msg)
                disp("No camera calibration file found. Plese run camera calibration");
            end          
        end
        
        % Returns an undistorted camera image
        function img = getImage(self)
            raw_img =  snapshot(self.cam);
            [img, new_origin] = undistortFisheyeImage(raw_img, self.params.Intrinsics, 'OutputView', 'full');
        end

        
        function [newIs, pose] = calculateCameraPos(self)  % DO NOT USE
            % calculateCameraPos Get transformation from camera to checkerboard frame
            % This function will get the camera position based on checkerboard.
            % You should run this function every time the camera position is changed.
            % It will calculate the extrinsics, and output to a transformation matrix.
            % Keep in mind: this transformation matrix is a transformation from pixels
            % to x-y coordinates in the checkerboard frame!

            % 1. Capture image from camera
            raw_img =  snapshot(self.cam);
            % 2. Undistort Image based on params
            [img, newIs] = undistortFisheyeImage(raw_img, self.params.Intrinsics, 'OutputView', 'full');
            % 3. Detect checkerboard in the image
            [imagePoints, boardSize] = detectCheckerboardPoints(img, 'PartialDetections', false);
            % 4. Compute transformation
            self.params.WorldPoints = self.params.WorldPoints(self.params.WorldPoints(:, 2) <= (boardSize(1)-1)*25, :);
            worldPointSize = size(self.params.WorldPoints);
            imagePointSize = size(imagePoints);
            fprintf("World Points is %d x %d\n", worldPointSize(1), worldPointSize(2));
            fprintf("Image Points is %d x %d\n", imagePointSize(1), imagePointSize(2));
            fprintf("The checkerboard is %d squares long x %d squares wide\n", boardSize(1), boardSize(2));

            % 4. Compute transformation
            [R, t] = extrinsics(imagePoints, self.params.WorldPoints, newIs);

            self.cam_R = R;
            self.cam_T = t;
            self.cam_TForm = rigid3d([ self.cam_R, zeros(3,1); self.cam_T, 1 ]);
            
            pose = [   R,    t';
                    0, 0, 0, 1];
        end

        % Takes a given image and mask function
        % Masks, erodes, and returns the regions left within the image
        function poi = getObjects(~, image, maskFcn)
            % Find the colors within the image
            masked = maskFcn(image);
        
            % Remove the speckles that are left over
            eroded = imerode(masked, strel("disk",2));
        
            % Get the properties of the remaining regions
            % Also filters the regions with small areas (likely speckles)
            regions = regionprops(eroded);
            objects = [];
            for index = 1:length(regions)
                if (regions(index).Area > 10)
                    objects = [objects; regions(index).Centroid, regions(index).Area];
                end
            end

            % Return if there are no objects
            if (size(objects) == 0)
                poi = [];
                return;
            end

            objects

            % Sort the objects based on area
            objects = sortrows(objects, 2);

            % Drop the area off of the objects
            poi = objects(:, 1);
        end
        
        % Converts a 2D camera coordinate to a 3D checkboard coordinate
        % Takes into account camera's angle of elevation and automatically
        % adjusts for it
        % If multiple points are specified, then the first one will be used
        function coord = poiToCoord(self, poi)
            if (size(poi, 2) == 0)
                error("No point specified")
            elseif (size(poi, 2) == 1)
                worldPt = pointsToWorld(self.getCameraIntrinsics(), self.getRotationMatrix(), self.getTranslationVector(), poi);
            else
                worldPt = pointsToWorld(self.getCameraIntrinsics(), self.getRotationMatrix(), self.getTranslationVector(), poi(1,:));
            end
            R_0_checker = [ 0  1  0; -1  0  0; 0  0 -1];
            t_0_checker = [90; 106; 0]; % Might need to be adjusted
            T_0_check = [R_0_checker, t_0_checker;zeros(1,3), 1];
            coord = inv(T_0_check) \ [worldPt'; 0; 1];
        
            % Coords are projected onto 2D checkerboard
            % We know the size of the balls and the camera position
            % Calculate the angle of elevation of the vector to the camera
            % Then fix the coordinates by those values
            ball_radius = 25/2;
            camera_pos = [365, 0, 180]; % wrt to the robot arm origin
        
            % Variables for easier readability
            x = 1;
            y = 2;
            z = 3;
        
            % Calculate the angle of elevation
            % We can assume that the Z coordinate from camera conversion will be 0
            angleOfElev = atan2(camera_pos(z), sqrt((camera_pos(x)-coord(x))^2 + (camera_pos(y)-coord(y))^2));
        
            % Calculate the angle in the XY plane
            % Since camera is at Y=0, ignore it in the calculations
            xyPlanarAngle = atan2(coord(y), camera_pos(x)-coord(x));
        
            % Correct the X and Y coordinates
            correction = ball_radius / tan(angleOfElev);
            coord(x) = coord(x) + correction * cos(xyPlanarAngle);
            coord(y) = coord(y) - correction * sin(xyPlanarAngle);
        
            % Set the ball's radius in the coordinate
            coord(z) = ball_radius;
        end % poisToCoords
    end
end