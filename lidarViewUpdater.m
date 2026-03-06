classdef lidarViewUpdater < handle
    properties   
        lidarViewer
        colors
        cuboidPlotHandles
        cmap
    end
    methods
        function obj = lidarViewUpdater(lidarViewer)
            obj.lidarViewer = lidarViewer;
            show(obj.lidarViewer);
            
            colorcubeCmap = colorcube;
            
            % Define labels to use for segmented points, specified as [R,G,B]
            obj.cmap = [0 0 0; ... % Unlabeled points
                1 1 1; ... % Ground points
                0 1 0; ... % ROI points
                0 0 1; ... % Obstacle points
                colorcubeCmap(randperm(size(colorcubeCmap,1)),:)];    
            
            % Set the colormap
            colormap(obj.lidarViewer.Axes, obj.cmap);
            
            % Define indices for each label
            obj.colors.Unlabeled    = 1;
            obj.colors.Ground       = 2;
            obj.colors.ROI          = 3;
            obj.colors.Obstacle     = 4;
            
            % initialize cuboid plot handles
            obj.cuboidPlotHandles = {};                 
        end
        
        function updateView(obj,ptCloud,labelIndices,varargin)
            %updateView update streaming point cloud display
            %   updates the pcplayer object specified in lidarViewer with a new point
            %   cloud ptCloud. Points specified in the struct labelIndices are colored
            %   according to the colormap of lidarViewer using the labelIndices specified by
            %   the struct colors. 
            [ptCloudHeight,ptCloudWidth,ptCloudDim] = size(ptCloud.Location);
            
            % Initialize colormap
            colormapValues = ones(ptCloudHeight * ptCloudWidth, ptCloudDim) .* obj.cmap(obj.colors.Unlabeled,:);
                        
            if isfield(labelIndices, 'ROI')
                colormapValues(labelIndices.ROI, :) = repmat(obj.cmap(obj.colors.ROI,:),sum(labelIndices.ROI,'all'),1);
            end
            
            if isfield(labelIndices, 'Ground')
                colormapValues(labelIndices.Ground, :) = repmat(obj.cmap(obj.colors.Ground,:),sum(labelIndices.Ground,'all'),1);
            end

            if isfield(labelIndices, 'Obstacle')
                colormapValues(labelIndices.Obstacle, :) = repmat(obj.cmap(obj.colors.Obstacle,:),sum(labelIndices.Obstacle,'all'),1);
            end

            if isfield(labelIndices, 'Cluster')    
                for i=1:numel(labelIndices.Cluster)
                    colormapValues(labelIndices.Cluster{i}, :) = repmat(obj.cmap(i+4,:),sum(labelIndices.Cluster{i},'all'),1);
                end
            end

            % Update view
            view(obj.lidarViewer, ptCloud.Location, reshape(colormapValues, ptCloudHeight,ptCloudWidth,ptCloudDim));

            % if detections are given as input, plot 3D bounding boxes
            if numel(varargin)
               for iHandle=1:numel(obj.cuboidPlotHandles)
                   delete(obj.cuboidPlotHandles{iHandle});
               end
               detections = varargin{1};
               numDetections = numel(detections);
               obj.cuboidPlotHandles = cell(1,numDetections);
               for iDet=1:numDetections
                  obj.cuboidPlotHandles{iDet} = plot(detections{iDet}, 'Parent', obj.lidarViewer.Axes); 
               end
               
            end
            
        end
    end
end
