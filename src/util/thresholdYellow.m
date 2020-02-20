function [BW,maskedRGBImage] = thresholdYellow(RGB)
%createMask  Threshold RGB image using auto-generated code from colorThresholder app.
%  [BW,MASKEDRGBIMAGE] = createMask(RGB) thresholds image RGB using
%  auto-generated code from the colorThresholder App. The colorspace and
%  minimum/maximum values for each channel of the colorspace were set in the
%  App and result in a binary mask BW and a composite image maskedRGBImage,
%  which shows the original RGB image values under the mask BW.

% Auto-generated by colorThresholder app on 20-Feb-2020
%------------------------------------------------------


% Convert RGB image to chosen color space
I = rgb2lab(RGB);

% Define thresholds for channel 1 based on histogram settings
channel1Min = 50.518;
channel1Max = 100.000;

% Define thresholds for channel 2 based on histogram settings
channel2Min = -31.407;
channel2Max = 54.668;

% Define thresholds for channel 3 based on histogram settings
channel3Min = 32.303;
channel3Max = 54.330;

% Create mask based on chosen histogram thresholds
sliderBW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
    (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
    (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
BW = sliderBW;

% Initialize output masked image based on input image.
maskedRGBImage = RGB;

% Set background pixels where BW is false to zero.
maskedRGBImage(repmat(~BW,[1 1 3])) = 0;

end
