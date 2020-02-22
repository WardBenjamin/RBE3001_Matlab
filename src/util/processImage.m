function [yMask, gMask, bMask, kMask] = processImage(image)

imageGrey = rgb2gray(image);

% Convert testImage from RGB color-space to LAB
imageLab = rgb2lab(image);

% Find binary masks for color regions
yMask = bwareaopen(thresholdYellow(image, imageLab), 300);
gMask = bwareaopen(thresholdGreen(image, imageLab), 300);
bMask = bwareaopen(thresholdBlue(image, imageLab), 300);
kMask = imerode(bwareaopen(thresholdBlack(imageGrey), 300), strel('disk', 8, 4));

end