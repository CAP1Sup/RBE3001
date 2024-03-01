try
    load("camParams.mat");
    disp("Loaded Camera Parameters from camParams.mat");
catch exception
    disp("Could not find camParams.mat, creating new Camera object");
    cam = Camera();
    save("camParams.mat","cam");
    disp("Saved Camera Parameters to camParams.mat");
end

image = cam.getImage();
"Yellow coords:  " + num2str(reshape(poiToCoord(cam, getObjects(image, @yellowMask))', 1, []))
%"Orange coords:  " + num2str(reshape(poiToCoord(cam, getObjects(image, @orangeMask))', 1, []))
%"Red coords:  " + num2str(reshape(poiToCoord(cam, getObjects(image, @redMask))', 1, []))
%"Green coords:  " + num2str(reshape(poiToCoord(cam, getObjects(image, @greenMask))', 1, []))
%"Light Green coords:  " + num2str(reshape(poiToCoord(cam, getObjects(image, @lightGreenMask))', 1, []))

function poi = getObjects(image, maskFcn)
    % Find the colors within the image
    masked = maskFcn(image);

    % Remove the speckles that are left over
    eroded = imerode(masked, strel("disk",2));

    % Get the centriods of the remaining regions
    regions = regionprops(eroded);
    poi = [];
    for index = 1:length(regions)
        poi = [poi; regions(index).Centroid];
    end
end

function coord = poiToCoord(cam, poi)
    if (size(poi, 2) == 0)
        error("No point specified")
    elseif (size(poi, 2) == 1)
        worldPt = pointsToWorld(cam.getCameraIntrinsics(), cam.getRotationMatrix(), cam.getTranslationVector(), poi);
    else
        worldPt = pointsToWorld(cam.getCameraIntrinsics(), cam.getRotationMatrix(), cam.getTranslationVector(), poi(1,:));
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