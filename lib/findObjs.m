function [yellowObjs, greenObjs, blueObjs, blackObjs, yRadii, gRadii, bRadii, yMask, gMask, bMask, kMask] = findObjs(imOrig, T_checker_to_robot, T_cam_to_checker, cameraParams)
% FINDOBJS implements a sequence of image processing steps to detect
% any objects of interest that may be present in an RGB image.
%
% Note: this function contains several un-implemented sections - it only
% provides a skeleton that you can use as reference in the development of
% your image processing pipeline. Feel free to edit as needed (or, feel
% free to toss it away and implement your own function).
%
%   Usage
%   -----
%   [IMDETECTEDOBJS, ROBOTFRAMEPOSE] = findObjs(IMORIG, TCHECKER2ROBOT, TCAM2CHECKER, CAMERAPARAMS)
%
%   Inputs
%   ------
%   IMORIG - an RGB image showing the robot's workspace (capture from a CAM
%   object).
%
%   TCHECKER2ROBOT - the homogeneous transformation matrix between the
%   checkered board and the reference frame at the base of the robot.
%
%   TCAM2CHECKER - the homogeneous transformation matrix between the camera
%   reference frame and the checkered board (you can calculate this using
%   the GETCAMTOCHECKERBOARD function, provided separately).
%
%   CAMERAPARAMS - an object containing the camera's intrinsic and
%   extrinsic parameters, as returned by MATLAB's camera calibration app.
%
%   Outputs
%   -------
%   Ideally, this function should return:
%   IMDETECTEDOBJS - a binarized image showing the location of the
%   segmented objects of interest.
%   
%   ROBOTFRAMEPOSE - the coordinates of the objects expressed in the robot's
%   reference frame
%
%   Authors
%   -------
%   Nathaniel Dennler  <nsdennler@wpi.edu>
%   Sean O'Neil        <stoneil@wpi.edu> 
%   Loris Fichera      <lfichera@wpi.edu>
%
%   Latest Revision
%   ---------------
%   2/12/2019


%%  1. First things first - undistort the image using the camera parameters
[im, ~] = undistortImage(imOrig, cameraParams, 'OutputView', 'full');

%%  2. Segment the image to find the objects of interest.

[yMask, gMask, bMask, kMask] = processImage(im);

[yCentr, yRadii] = findObjLocations(yMask);
[gCentr, gRadii] = findObjLocations(gMask);
[bCentr, bRadii] = findObjLocations(bMask, true);

[yellowObjs, greenObjs, blueObjs, blackObjs] = findObjSizes(yCentr, gCentr, bCentr, kMask);

% You can easily convert image pixel coordinates to 3D coordinates (expressed in the
% checkerboard reference frame) using the following transformations:

R = T_cam_to_checker(1:3,1:3);
t = T_cam_to_checker(1:3,4);

% TODO: Use object classes because matrix operations are hard
if ~isempty(yellowObjs)
    disp('Yellow');
    yellowObjs = horzcat(horzcat(cameraToWorld(yellowObjs(:,1:2), cameraParams, R, t, T_checker_to_robot), yellowObjs(:,end)), yellowObjs(:,1:2));
    for idx = 1:length(yellowObjs(:, 1))
        yellowObjs(idx, 2) = yellowObjs(idx, 2) + 222;
    end
end

if ~isempty(greenObjs)
    disp('Green');
    greenObjs = horzcat(horzcat(cameraToWorld(greenObjs(:,1:2), cameraParams, R, t, T_checker_to_robot), greenObjs(:,end)), greenObjs(:,1:2));
    for idx = 1:length(greenObjs(:, 1))
        greenObjs(idx, 2) = greenObjs(idx, 2) + 222;
    end
end

if ~isempty(blueObjs)
    disp('Blue');
    blueObjs = horzcat(horzcat(cameraToWorld(blueObjs(:,1:2), cameraParams, R, t, T_checker_to_robot), blueObjs(:,end)), blueObjs(:,1:2));
    for idx = 1:length(blueObjs(:, 1))
        blueObjs(idx, 2) = blueObjs(idx, 2) + 222;
    end
end
% see https://www.mathworks.com/help/vision/ref/cameraparameters.pointstoworld.html
% for details on the expected dimensions for YOUR_PIXEL_VALUES)
end

function worldCentroids = cameraToWorld(cameraCentroids, cameraParams, R, t, T_checker_to_robot)
    checkCentroids = pointsToWorld(cameraParams, R, t, cameraCentroids);

    [len, ~] = size(checkCentroids);
    worldCentroids = horzcat(horzcat(checkCentroids, zeros(len,1)), ones(len,1)).';
    
    for i = 1:len
        worldCentroids(1:end, i) = T_checker_to_robot * worldCentroids(1:end, i)
    end
    
    worldCentroids = worldCentroids.';
    worldCentroids = worldCentroids(:,1:2);
end