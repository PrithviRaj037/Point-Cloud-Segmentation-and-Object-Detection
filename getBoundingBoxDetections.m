function [detections] = getBoundingBoxDetections(ptCloud,clusterIndices,maxLength,maxWidth)
    % This method fits bounding boxes on each cluster with some basic
    % rules.
    % Cluster must have at least minDetsPerCluster points.
    % Its mean z must be between maxZDistance and minZDistance.
    % length, width and height are calculated using min and max from each
    % dimension.
    numClusters = numel(clusterIndices);
    detections = cell(1,numClusters);
    isValidDetection = false(1,numClusters);
    
    for i = 1:numClusters
        cuboid = pcfitcuboid(ptCloud,find(clusterIndices{i}));
        
%         yaw = cuboid.Orientation(3);
%         L = cuboid.Dimensions(1);
%         W = cuboid.Dimensions(2);
%         H = cuboid.Dimensions(3);
%         if abs(yaw) > 45
%             possibles = yaw + [-90;90];
%             [~,toChoose] = min(abs(possibles));
%             yaw = possibles(toChoose);
%             temp = L;
%             L = W;
%             W = temp;
%         end
%         detections{i} = cuboidModel([cuboid.Center,L,W,H,cuboid.Orientation(1),cuboid.Orientation(2),yaw]);
        
        detections{i} = cuboid;
        isValidDetection(i) = cuboid.Dimensions(1) < maxLength & cuboid.Dimensions(2) < maxWidth;
    end
    detections = detections(isValidDetection);
end

