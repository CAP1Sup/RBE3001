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
"Orange coords:  " + num2str(reshape(poiToCoord(cam, getObjects(image, @orangeMask))', 1, []))
"Red coords:  " + num2str(reshape(poiToCoord(cam, getObjects(image, @redMask))', 1, []))
"Green coords:  " + num2str(reshape(poiToCoord(cam, getObjects(image, @greenMask))', 1, []))
"Light Green coords:  " + num2str(reshape(poiToCoord(cam, getObjects(image, @lightGreenMask))', 1, []))

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
    if (size(poi, 2) == 1)
        worldPt = pointsToWorld(cam.getCameraIntrinsics(), cam.getRotationMatrix(), cam.getTranslationVector(), poi);
    else
        worldPt = pointsToWorld(cam.getCameraIntrinsics(), cam.getRotationMatrix(), cam.getTranslationVector(), poi(1,:));
    end
    R_0_checker = [ 0  1  0; 1  0  0; 0  0 -1];
    t_0_checker = [111; -70; 0]; % Might need to be adjusted
    T_0_check = [R_0_checker, t_0_checker;zeros(1,3), 1];
    coord = inv(T_0_check) \ [worldPt'; 0; 1];
end % poisToCoords