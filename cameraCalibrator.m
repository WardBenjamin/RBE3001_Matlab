% Auto-generated by cameraCalibrator app on 19-Feb-2020
%-------------------------------------------------------


% Define images to process
imageFileNames = {'/ifs/home/blward/My_Documents/RBE3001/matlab/src/Image1.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/src/Image2.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/src/Image4.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/src/Image5.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/src/Image6.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/src/Image7.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/src/Image9.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/src/Image10.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/src/Image12.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/src/Image15.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/src/Image16.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/src/Image17.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/src/Image18.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/src/Image19.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/src/Image20.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/src/Image21.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/src/Image22.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/src/Image23.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/src/Image24.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/src/Image25.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/src/Image27.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/src/Image29.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/src/Image30.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/src/Image31.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/src/Image32.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/src/Image33.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/src/Image34.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/src/Image35.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/src/Image37.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/src/Image41.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/src/Image42.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/src/Image43.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/src/Image44.png',...
    '/ifs/home/blward/My_Documents/RBE3001/matlab/src/Image47.png',...
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
