robot = Robot();

% Setup camera
try
    load("camParams.mat");
    disp("Loaded Camera Parameters from camParams.mat");
    load("checker.mat")
catch exception
    disp("Could not find camParams.mat, creating new Camera object");
    cam = Camera();
    save("camParams.mat","cam");
    disp("Saved Camera Parameters to camParams.mat");
end

% Define masks and their respective sorting destinations in form:
% [mask function, [x,y,z]]
% Alpha is assumed to be 90 deg (gripper vertical)
possible_objects = {@yellowMask, [250, 140, 50.5];
                    @redMask, [250,70,45.5];
                    @greenMask [250,-70,40.5];
                    @orangeMask, [250, -140, 50.5];
                    @lightGreenMask, [100,190,50.5];
                    %@greyMask, [-140,210,17.5]
                    } ;
%coordinates of the checkerboard
x = [412 848 1000 295];
y = [323 318 495 512];

% Named indexes for easier readability when using possible_objects
mask_i = 1;
sort_pos_i = 2;
% binary that only takes in the checkerboard coords
BW = poly2mask(x,y,642,1080);
% Define a standby pose
% Arm will wait here until an object is detected
standby_pose = [130, 0, 130, 90]; % [mm, mm, mm, deg]

% Define a default move time
% Will be used for all arm movements between points
% Start out slow... then once we have it working...
% MAXIMUM SPEED ;-)
travelTime = 0.5; % s

% Move to the home position
% Standardizes starting position
robot.set_joint_vars([0,0,0,0], 3000);
pause(3)

% Move to the "standby" pose and open gripper
robot.quintic_move(standby_pose, travelTime);
robot.writeGripper(true);

% Main interation loop
while (true)
    % Get an image from the camera
    image = cam.getImage;

    % Mask out everything but the checkerboard
    image = bsxfun(@times, image, cast(BW,'like',image));

    % Define variables to be used if the loop is successful
    coords = [];
    sort_pos = [];
    
    % Loop through masks, looking for objects
    for index = 1:length(possible_objects)
        pois = cam.getObjects(image, possible_objects{index,mask_i});
        if (size(pois, 2) > 0)
            coords = cam.poiToCoord(pois);
            sort_pos = possible_objects{index,sort_pos_i};
            break;
        end
    end

    % If there were no objects, restart the loop
    if (size(coords, 2) == 0)
        robot.simple_quintic_move(standby_pose, travelTime);
        continue;
    end

    % Adjust the Z value of the coordinates (robot arm would crash)
    coords(3) = coords(3) + 50;

    % Set the alpha of the coords to be 90 (wrist down)
    coords(4) = 90;
    
    % Move to above the ball
    robot.simple_quintic_move(transpose(coords), travelTime);

    % Lower down and pick up the ball
    coords(3) = coords(3) - 35; % Should be 15 mm above the ball's coords
    robot.simple_quintic_move(transpose(coords), 0.5);
    robot.writeGripper(false);

    % Move upwards to clear the balls
    coords(3) = coords(3) + 50;
    robot.simple_quintic_move(transpose(coords), travelTime);

    % Move to the sort position of the object
    robot.simple_quintic_move([sort_pos, 60], travelTime);

    % Drop the object
    robot.writeGripper(true);

    % Move up above the sort position
    sort_pos(3) = sort_pos(3) + 20;
    robot.simple_quintic_move([sort_pos, 60], travelTime);
end % main iteration loop