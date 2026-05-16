function [ptCloudOut,obstacleIndices,groundIndices] = removeGroundPlane(ptCloudIn,maxZHeight,maxGroundDist,referenceVector,maxAngularDist,currentIndices)
    % This method removes the ground plane from point cloud using
    % pcfitplane.
    locations = ptCloudIn.Location;
    belowZ = locations(:,:,3) < maxZHeight;
    
    
    [~,inlierIndices,outlierIndices] = pcfitplane(select(ptCloudIn,belowZ,'OutputSize','full'),maxGroundDist,referenceVector,maxAngularDist);
    ptCloudOut = select(ptCloudIn,outlierIndices, 'OutputSize', 'full');
    
    tmpIndices = false(size(currentIndices));
    tmpIndices(outlierIndices) = true;    
    obstacleIndices = currentIndices & tmpIndices;
    
    tmpIndices = false(size(currentIndices));
    tmpIndices(inlierIndices) = true;
    groundIndices = currentIndices & tmpIndices;
end
