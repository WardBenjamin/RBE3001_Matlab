[coms, cameraParams, T_base_check] = initialize();

T_cam_check = load('camToCheckTrans.mat'); T_cam_check = T_cam_check.T_cam_check;
testImage = imread('image_pipeline_test.png');

yMask = thresholdYellow(testImage);
gMask = thresholdGreen(testImage);
bMask = thresholdBlue(testImage);

[yC, yR] = findObjects(yMask);
[gC, gR] = findObjects(gMask);
[bC, bR] = findObjects(bMask, true);

figure(1);
imshow(testImage);
hold on;
viscircles(yC, yR, 'Color', 'y', 'LineWidth', 4);
viscircles(gC, gR, 'Color', 'g', 'LineWidth', 4);
viscircles(bC, bR, 'Color', 'b', 'LineWidth', 4);

function [centroids, radii] = findObjects(mask, filterSmall)
    openedMask = bwareaopen(mask, 300);
    stats = regionprops('table', openedMask, 'Centroid', 'MajorAxisLength', 'MinorAxisLength')
    centroids = stats.Centroid;
    diameters = mean([stats.MajorAxisLength stats.MinorAxisLength],2);
    
    
    if nargin > 1
        % TODO: This would be faster if it was inverted, I think
        for idx = 1:length(centroids)
            % Arbitrary threshold found through inspection
            if stats.MajorAxisLength(idx)/stats.MinorAxisLength(idx) > 1.2 
                centroids(idx, :) = [];
                diameters(idx) = [];
            end
        end
    end
    radii = diameters/2;
end