clear;
rng(3);
%% Track Vehicles Using Lidar: From Point Cloud to Track List
% This example shows you how to detect objects using measurements from a
% lidar sensor mounted on top of an ego vehicle. Lidar sensors report
% measurements as a point cloud. The example illustrates the workflow in
% MATLAB(R) for processing the point cloud and detecting the objects. 
% The lidar data used in this example is recorded from a highway driving 
% scenario. In this example, you use the recorded data to detect vehicles
% by the following steps:
% 1. Crop point cloud to the region of interest
% 2. Removing the ground points
% 3. Cluster the remaining points
% 4. Fitting a bounding box (cuboid) on the clusters

% The script is based on the Matlab examples
% https://de.mathworks.com/help/fusion/ug/track-vehicles-using-lidar.html
% and https://de.mathworks.com/help/driving/ug/ground-plane-and-obstacle-detection-using-lidar.html

%% 3-D Bounding Box Detector Model
% Due to high resolution capabilities of the lidar sensor, each scan from
% the sensor contains a large number of points, commonly known as a point
% cloud. This raw data must be preprocessed to extract objects of interest,
% such as cars, cyclists, and pedestrians. For this, the point cloud data
% must be segmented by identifying the ground points as well as obstacles
% In this example, the point clouds belonging to obstacles are further 
% classified into clusters using the |segmentLidarData| function, and each 
% cluster is converted to a bounding box detection with the following format:
%
% $[x\ y\ z\ l\ w\ h]$
%
% $x$, $y$ and $z$ refer to the x-, y- and z-positions of the bounding
% box and $l$, $w$ and $h$ refer to its length, width, and height,
% respectively.
%
% The bounding box is fit onto each cluster by using minimum and maximum of
% coordinates of points in each dimension. The detector is implemented by a
% supporting class |BoundingBoxDetector|, which wraps around point
% cloud segmentation and clustering functionalities. An object of this
% class accepts a |pointCloud| input and returns a list of
% |objectDetection| objects with bounding box measurements.

%% 1) Load Lidar Data
% The lidar data used in this example was recorded using a Ouster OS1-64
% sensor mounted on a vehicle. Load data if unavailable. The lidar data is 
% stored as a cell array of pointCloud objects.
if ~exist('pointClouds','var')
    % Specify initial and final time for simulation.
    pointClouds = load('pointClouds_highway.mat');
    pointClouds = pointClouds.pointClouds;
end

%% 2) Instantiate Point Cloud Player
% Each scan of lidar data is stored as a 3-D point cloud. Efficiently
% processing this data using fast indexing and search is key to the
% performance of the sensor processing pipeline. This efficiency is
% achieved using the pointCloud object, which internally organizes the
% data using a K-d tree data structure. 
%
% The Location property of the pointCloud is an M-by-N-by-3
% matrix, containing the XYZ coordinates of points in meters.

% The pcplayer can be used to visualize streaming point cloud data. Setup
% the region around the vehicle to display by configuring
% pcplayer.

% Specify limits of point cloud display
xlimits = [-60 60]; % meters
ylimits = [-30 30];
zlimits = [-5 15];

% Create a pcplayer
lidarViewer = pcplayer(xlimits, ylimits, zlimits, 'MarkerSize', 0.5);

% Customize player axes labels
xlabel(lidarViewer.Axes, 'X (m)')
ylabel(lidarViewer.Axes, 'Y (m)')
zlabel(lidarViewer.Axes, 'Z (m)')

%% Visualize Point Cloud
% Visualize every scan
for iScan = 1:numel(pointClouds)
    view(lidarViewer, pointClouds{iScan});
    pause(0.05);
    if ~isOpen(lidarViewer)
        break;
    end
end


%% Initialize lidarViewUpdater
viewUpdater = lidarViewUpdater(lidarViewer);

%% 4) Inspect single scan
% In this example, we will be segmenting points belonging to the ground
% plane and nearby obstacles. Set the colormap for
% labeling these points.

% Extract scan and visualize it in pcplayer
ptCloud = pointClouds{150};
view(lidarViewer, ptCloud);

%% 5) Crop Point Cloud to Region of Interest (ROI) and visualize it
% By inspecting the Point Cloud scans of the highway drive, it is visible
% that only objects (vehicles, traffic signs, ...) which are closely around 
% the ego vehicle can be really identified. At higher ranges the point 
% cloud density becomes too low to detect objects. Therefore, it is
% reasonable to crop the point cloud to a ROI closely around the ego
% vehicle. Write a function |cropPointCloud| that is realizing the cropping
% using logical indexing of the point locations (ptCloud.Location).

% edit cropPointCloud.m

xROI = [-40, 40];
yROI = [-12.5, 12.5];
zROI = [-10, 10];

[ptCloudROI,labelIndices.ROI,croppedlabelIndices] = cropPointCloud(ptCloud,xROI,yROI,zROI);

% Visualize the cropped point cloud
updateView(viewUpdater, ptCloud, labelIndices);

%% 6) Segment Ground Plane and Nearby Obstacles
% In order to identify obstacles from the lidar data, the ground plane 
% needs to be segmented first and removed from the data.
% Write a function |removeGroundPlane| to accomplish this. It should be
% based on |segmentGroundFromLidarData| which segments points belonging to 
% the ground from organized lidar data.

% edit removeGroundPlane.m

% maximum z height
maxZHeight = -1;
% GroundMaxDistance Maximum distance of point to the ground plane
maxGroundDist = 0.2;
% GroundReferenceVector Reference vector of ground plane
referenceVector = [0 0 1];
% GroundMaxAngularDistance Maximum angular distance of point to reference vector
maxAngularDist = 5;

[ptCloudObstacle,labelIndices.Obstacle,labelIndices.Ground] = removeGroundPlane(ptCloudROI,maxZHeight,maxGroundDist,referenceVector,maxAngularDist,labelIndices.ROI);

% Visualize the segmented ground points
updateView(viewUpdater, ptCloud, labelIndices);

%% 7) Segment Obstacle Points into Clusters

% edit segmentObstaclePointsIntoClusters.m

minDistance = 1.5;
minDetsPerCluster = 30;

[labelIndices.Cluster] = segmentObstaclePointsIntoClusters(ptCloudObstacle,minDistance,minDetsPerCluster);

updateView(viewUpdater, ptCloud, labelIndices);

%% 8) get bounding box detections
maxLength = 15;
maxWidth = 5;

[detections] = getBoundingBoxDetections(ptCloudObstacle,labelIndices.Cluster,maxLength,maxWidth);

updateView(viewUpdater, ptCloud, labelIndices, detections);


%% Loop over complete test drive

for iScan = 1:numel(pointClouds)
    
    ptCloud = pointClouds{iScan};
    
    [ptCloudROI,labelIndices.ROI,croppedlabelIndices] = cropPointCloud(ptCloud,xROI,yROI,zROI);
    
    [ptCloudObstacle,labelIndices.Obstacle,labelIndices.Ground] = removeGroundPlane(ptCloudROI,maxZHeight,maxGroundDist,referenceVector,maxAngularDist,labelIndices.ROI);
    
    [labelIndices.Cluster] = segmentObstaclePointsIntoClusters(ptCloudObstacle,minDistance,minDetsPerCluster);
    
    [detections] = getBoundingBoxDetections(ptCloudObstacle,labelIndices.Cluster,maxLength,maxWidth);
    
    updateView(viewUpdater, ptCloud, labelIndices, detections);
    
    pause(0.01);

    if ~isOpen(lidarViewer)
        break;
    end

end
%%


%% References
% [1] Arya Senna Abdul Rachman, Arya. "3D-LIDAR Multi Object Tracking for
% Autonomous Driving: Multi-target Detection and Tracking under Urban Road
% Uncertainties." (2017).