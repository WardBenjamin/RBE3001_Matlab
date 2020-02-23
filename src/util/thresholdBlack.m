function [BW,maskedImage] = thresholdBlack(X)
%segmentImage Segment image using auto-generated code from imageSegmenter App
%  [BW,MASKEDIMAGE] = segmentImage(X) segments image X using auto-generated
%  code from the imageSegmenter App. The final segmentation is returned in
%  BW, and a masked image is returned in MASKEDIMAGE.

% Auto-generated by imageSegmenter app on 23-Feb-2020
%----------------------------------------------------


% Threshold image - manual threshold
BW = X > 57;

% Polygon drawing
xPos = [83.5622 -0.6483 1.6483 640.8828 640.8828 0.8828 0.1172 78.9689 46.0502 578.8732 490.8349 441.0742 418.1077 348.4426 191.5048 109.5909];
yPos = [242.7967 245.0933 0.8828 0.8828 480.1172 480.1172 246.6244 246.6244 435.7153 420.4043 95.0455 95.8110 105.7632 103.4665 111.8876 132.5574];
m = size(BW, 1);
n = size(BW, 2);
addedRegion = poly2mask(xPos, yPos, m, n);
BW = BW | addedRegion;

% Rectangle drawing
xPos = [40.6914 85.8589 85.8589 40.6914];
yPos = [234.3756 234.3756 251.2177 251.2177];
m = size(BW, 1);
n = size(BW, 2);
addedRegion = poly2mask(xPos, yPos, m, n);
BW = BW | addedRegion;

% Invert mask
BW = imcomplement(BW);

% Fill holes
BW = imfill(BW, 'holes');

% Create masked image.
maskedImage = X;
maskedImage(~BW) = 0;