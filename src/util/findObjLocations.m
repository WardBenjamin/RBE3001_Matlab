function [centroids, radii] = findObjLocations(mask, filterSmall)
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