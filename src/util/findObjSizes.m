function [yellowObjects, greenObjects, blueObjects, blackObjects] = findObjSizes(yC, gC, bC, kMask)
[objSize, blackObjects] = findObjectSize([yC; gC; bC], kMask);

yEnd = length(yC) / 2;
gEnd = length(gC) / 2 + yEnd;

yellowObjects = [yC objSize(1:yEnd, 1)];
greenObjects = [gC objSize(yEnd + 1:gEnd, 1)];
blueObjects = [bC objSize(gEnd + 1:end, 1)];

end

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
