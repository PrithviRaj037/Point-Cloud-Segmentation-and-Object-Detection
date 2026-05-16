function [clusterIndices] = segmentObstaclePointsIntoClusters(ptCloudIn,minDistance,minDetsPerCluster)
    % This method segments the obstacle points into clusters using
    % Euclidean distance segmentation (|pcsegdist|).
    % Cluster must have atleast minDetsPerCluster points.
    
    [labels,numClusters] = pcsegdist(ptCloudIn,minDistance);
    
    clusterIndices = cell(1, numClusters);
    isValidCluster = false(1,numClusters);
    
    for i = 1:numClusters
        clusterIndices{i} = labels == i;
        isValidCluster(i) = sum(clusterIndices{i},'all') >= minDetsPerCluster;
    end
    
    clusterIndices = clusterIndices(isValidCluster);
 
end
