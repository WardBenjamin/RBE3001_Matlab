% Auto-generated by cameraCalibrator app on 19-Feb-2020
%-------------------------------------------------------


% Define images to process
imageFileNames = {'/ifs/home/blward/My_Documents/RBE3001/matlab/Image4.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/Image5.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/Image8.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/Image9.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/Image10.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/Image11.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/Image13.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/Image14.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/Image20.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/Image21.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/Image22.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/Image23.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/Image24.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/Image25.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/Image26.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/Image31.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/Image32.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/Image35.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/Image38.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/Image40.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/Image41.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/Image42.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/Image44.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/Image45.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/Image46.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/Image48.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/Image50.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/Image51.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/Image52.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/Image56.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/Image57.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/Image58.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/Image59.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/Image60.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/Image61.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/Image62.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/Image63.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/Image64.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/Image65.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/Image67.png',...
    };

% Detect checkerboards in images
[imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(imageFileNames);
imageFileNames = imageFileNames(imagesUsed);

% Read the first image to obtain image size
originalImage = imread(imageFileNames{1});
[mrows, ncols, ~] = size(originalImage);

% Generate world coordinates of the corners of the squares
squareSize = 12;  % in units of 'millimeters'
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% Calibrate the camera
[cameraParams, imagesUsed, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...
    'EstimateSkew', false, 'EstimateTangentialDistortion', false, ...
    'NumRadialDistortionCoefficients', 2, 'WorldUnits', 'millimeters', ...
    'InitialIntrinsicMatrix', [], 'InitialRadialDistortion', [], ...
    'ImageSize', [mrows, ncols]);

% View reprojection errors
h1=figure; showReprojectionErrors(cameraParams);

% Visualize pattern locations
h2=figure; showExtrinsics(cameraParams, 'CameraCentric');

% Display parameter estimation errors
displayErrors(estimationErrors, cameraParams);

% For example, you can use the calibration data to remove effects of lens distortion.
undistortedImage = undistortImage(originalImage, cameraParams);

% See additional examples of how to use the calibration data.  At the prompt type:
% showdemo('MeasuringPlanarObjectsExample')
% showdemo('StructureFromMotionExample')
