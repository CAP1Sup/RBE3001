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
"CheckerBoard:  " 
"Yellow coords:  " + num2str(reshape(cam.poiToCoord(cam.getObjects(image, @yellowMask))', 1, []))
"Orange coords:  " + num2str(reshape(poiToCoord(cam, getObjects(image, @orangeMask))', 1, []))
"Red coords:  " + num2str(reshape(poiToCoord(cam, getObjects(image, @RedMask))', 1, []))
"Green coords:  " + num2str(reshape(poiToCoord(cam, getObjects(image, @GreenMask))', 1, []))
"Light Green coords:  " + num2str(reshape(poiToCoord(cam, getObjects(image, @lightGreenMask))', 1, []))