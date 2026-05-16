function [ptCloudOut,indices,croppedIndices] = cropPointCloud(ptCloudIn,xLim,yLim,zLim)
    % This method selects the point cloud within limits
    locations = ptCloudIn.Location;
    insideX = locations(:,:,1) < xLim(2) & locations(:,:,1) > xLim(1);
    insideY = locations(:,:,2) < yLim(2) & locations(:,:,2) > yLim(1);
    insideZ = locations(:,:,3) < zLim(2) & locations(:,:,3) > zLim(1);
    
%     % remove ego vehicle points
%     ranges = sqrt(locations(:,:,1).^2 + locations(:,:,2).^2 + locations(:,:,3).^2);
%     outsideR = ranges > minRadius;
    
    indices = insideX & insideY & insideZ;% & outsideR;
    croppedIndices = ~indices;
    ptCloudOut = select(ptCloudIn,indices, 'OutputSize', 'full'); % keep organized point cloud
    
end

