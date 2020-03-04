function [centroids, radii] = findObjLocations(mask, filterSmall)
% findObjLocations Finds the camera-space location of thresholded objects
%
%   Authors
%   -------
%   Benjamin Ward      <blward@wpi.edu>
%   Teresa Saddler     <tsaddler@wpi.edu>
%
%   Latest Revision
%   ---------------
%   03/04/2020

    stats = regionprops('table', mask, 'Centroid', 'MajorAxisLength', 'MinorAxisLength');
    centroids = stats.Centroid;
    diameters = mean([stats.MajorAxisLength stats.MinorAxisLength],2);
    
    if nargin > 1
        % This would be faster if it was inverted, I think, but works as-is
        offset = 0;
        for idx = 1:size(centroids, 1)
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