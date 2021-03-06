clear;

% Load test image
% testImage = imread('image_pipeline_test.png');
% testImageGrey = rgb2gray(testImage);
% In practice: cam = webcam(); image = snapshot(cam);
cam = webcam(); testImage = snapshot(cam); testImageGrey = rgb2gray(testImage);

% Convert testImage from RGB color-space to LAB
testImageLab = rgb2lab(testImage);

% Find binary masks for color regions
yMask = bwareaopen(thresholdYellow(testImage, testImageLab), 300);
gMask = bwareaopen(thresholdGreen(testImage, testImageLab), 300);
bMask = bwareaopen(thresholdBlue(testImage, testImageLab), 300);
kMask = imerode(bwareaopen(thresholdBlack(testImageGrey), 300), strel('disk', 8, 4));

% Segment image and remove false positives
[yC, yR] = findObjects(yMask);
[gC, gR] = findObjects(gMask);
[bC, bR] = findObjects(bMask, true);
[objSize, foundObjects] = findObjectSize([yC; gC; bC], kMask)

[yellowObjects, greenObjects, blueObjects, blackObjects] = findObjSizes(yC,gC,bC,kMask);

% Display detected objects
figure(1);
imshow(testImage);
hold on;
viscircles(yC, yR, 'Color', 'y', 'LineWidth', 4);
viscircles(gC, gR, 'Color', 'g', 'LineWidth', 4);
viscircles(bC, bR, 'Color', 'b', 'LineWidth', 4);
viscircles(foundObjects(:, 1:2), foundObjects(:, 3), 'Color', 'k');

function [objSize, foundObjects] = findObjectSize(coloredCentroids, kMask)
    % Find black regions near colored regions
    % Determine whether object is "large" or "small"
    stats = regionprops('table', kMask, 'Centroid', 'MajorAxisLength', 'MinorAxisLength');
    kCentroids = stats.Centroid;
    [nearestIdx, dist] = dsearchn(kCentroids, coloredCentroids)
    
    foundObjects = zeros(length(nearestIdx), 3);
    objSize = zeros(length(nearestIdx), 1);
    for idx = 1:length(nearestIdx)
        % diaMean = mean([stats.MajorAxisLength(nearestIdx(idx)),
        % stats.MinorAxisLength(nearestIdx(idx))],2);
        objDia = stats.MajorAxisLength(nearestIdx(idx));
        foundObjects(idx, :) = [kCentroids(nearestIdx(idx), :) objDia/2];
        objSize(idx) = objDia > 100;
    end
    
    
end

function [centroids, radii] = findObjects(mask, filterSmall)
    stats = regionprops('table', mask, 'Centroid', 'MajorAxisLength', 'MinorAxisLength');
    centroids = stats.Centroid;
    diameters = mean([stats.MajorAxisLength stats.MinorAxisLength],2);
    
    if nargin > 1
        % TODO: This would be faster if it was inverted, I think
        offset = 0;
%         while idx <= length(centroids)
%             % Arbitrary threshold found through inspection
%             sizeRatio = stats.MajorAxisLength(idx)/stats.MinorAxisLength(idx)
%             if sizeRatio > 1.2 
%                 centroids(idx, :) = [];
%                 diameters(idx) = [];
%             else
%                 idx = idx + 1;
%             end
%         end
        for idx = 1:length(centroids)
            % Arbitrary threshold found through inspection
            if stats.MajorAxisLength(idx)/stats.MinorAxisLength(idx) > 1.2 
                centroids(idx - offset, :) = [];
                diameters(idx - offset) = [];
                offset = offset + 1;
            end
        end
    end
    radii = diameters/2;
end