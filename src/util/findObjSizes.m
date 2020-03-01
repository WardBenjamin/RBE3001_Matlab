function [yellowObjects, greenObjects, blueObjects, blackObjects] = findObjSizes(yC, gC, bC, kMask)
[objSize, blackObjects] = findObjectSize([yC; gC; bC], kMask);

% yEnd = length(yC);
% 
% if yEnd == 0
%     yellowObjects = [];
%     
%     gEnd = length(gC);
%     
%     if gEnd ~= 0
%         greenObjects = [gC objSize(yEnd + 1:gEnd, 1)];
%     else
%         greenObjects = [];
%     end
%     
%     blueObjects = [bC objSize(gEnd + 1:end, 1)];
% else
%     yellowObjects = [yC objSize(1:yEnd, 1)];
%     
%     gEnd = length(gC) + yEnd;
%     
%     if gEnd ~= yEnd
%         gEnd = gEnd - 1;
%         greenObjects = [gC objSize(yEnd + 1:gEnd, 1)];
%     else
%         greenObjects = [];
%     end
%     
%     blueObjects = [bC objSize(gEnd + 1:end, 1)];
% end

% yEnd = length(yC) / 2;
% gEnd = length(gC) / 2 + yEnd;
% bEnd = length(bC) / 2 + gEnd;

yLen = 0;
gLen = 0;
bLen = 0;

if ~isempty(yC)
    yLen = length(yC(1:end, 1))
end
if ~isempty(gC)
    gLen = length(gC(1:end, 1))
end
if ~isempty(bC)
    bLen = length(bC(1:end, 1))
end

yellowObjects = [];
greenObjects = [];
blueObjects = [];

if yLen == 0
    if gLen ~= 0
        greenObjects = [gC objSize(1:gLen, 1)];
    end
    
    if bLen ~= 0
        blueObjects = [bC objSize(gLen + 1:end, 1)];
    end
else
    yellowObjects = [yC objSize(1:yLen, 1)];
    
    gEnd = gLen + yLen;
    
    if gLen ~= 0
        greenObjects = [gC objSize(yLen + 1:gEnd, 1)];
    end
    
    if bLen ~= 0
        blueObjects = [bC objSize(gEnd + 1:end, 1)];
    end
end

% yellowObjects = [yC objSize(1:yEnd, 1)]
% greenObjects = [gC objSize(yEnd + 1:gEnd, 1)]
% blueObjects = [bC objSize(gEnd + 1:end, 1)]

% yellowObjects = [yC objSize(1:yEnd, 1)];
% greenObjects = [gC objSize(yEnd + 1:gEnd, 1)];
% blueObjects = [bC objSize(gEnd + 1:end, 1)];

end

function [objSize, foundObjects] = findObjectSize(coloredCentroids, kMask)
    % Find black regions near colored regions
    % Determine whether object is "large" or "small"
    
    if isempty(coloredCentroids)
        objSize = [];
        foundObjects = [];
        return;
    end
    
    stats = regionprops('table', kMask, 'Centroid', 'MajorAxisLength', 'MinorAxisLength');
    kCentroids = stats.Centroid;
    [nearestIdx, dist] = dsearchn(kCentroids, coloredCentroids)
    
    foundObjects = zeros(length(nearestIdx), 3);
    objSize = zeros(length(nearestIdx), 1);
    for idx = 1:length(nearestIdx)
%         objDia = max([stats.MajorAxisLength(nearestIdx(idx)), stats.MinorAxisLength(nearestIdx(idx))]);
        objDia = stats.MajorAxisLength(nearestIdx(idx));
        objMinorDia = stats.MinorAxisLength(nearestIdx(idx));
        foundObjects(idx, :) = [kCentroids(nearestIdx(idx), :) objDia/2];
        objSize(idx) = (objDia > 100) & (objMinorDia > 60); % Prev: 100; didn't work for 
    end
    
    
end