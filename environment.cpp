/* \author Lucas Raimundo */
// Create simple 3d highway enviroment using PCL
// And loading PCL data
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
#include "processPointClouds.cpp"
#include <unordered_set>

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar* lidar = new Lidar (cars,0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
    //renderRays(viewer,lidar->position,inputCloud);
    renderPointCloud(viewer, inputCloud,"inputCloud");

    // TODO:: Create point processor

    // 1 will be rendered, 0 will not
    int render_obst = 1;
    int render_plane = 1;
    int render_clusters = 1;
    int render_box = 1;

    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane (inputCloud, 100, 0.2);
    
    if(render_obst)
        renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(0,1,1));
    if(render_plane)
        renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));
    
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 1.0, 2, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color (1,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        if(render_clusters)
        {
            std::cout << "cluster size ";
            pointProcessor.numPoints(cluster);
            renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        }
        
        if (render_box)
        {
            Box box = pointProcessor.BoundingBox(cluster);
            renderBox(viewer,box,clusterId);
        }

       
        ++clusterId;
    }

}

void builtIncityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI> pointProcessor, pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud)
//void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block ---------
    // ----------------------------------------------------

    // 1 will be rendered, 0 will not
    int render_obst = 1;
    int render_plane = 1;
    int filteredCloud = 0;
    int render_clusters = 1;
    int render_box = 1;

    // Create pointProcessor
    //ProcessPointClouds<pcl::PointXYZI> pointProcessor;
    // Create pointProcessorI
    //ProcessPointClouds<pcl::PointXYZI> pointProcessorI;

    // Load data into inputCloud
    //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessor.loadPcd("/home/workspace/SFND_Lidar_Obstacle_Detection/src/sensors/data/pcd/data_1/0000000000.pcd");
    // Render cloud
    //renderPointCloud(viewer,inputCloud,"cloud");

    // Create filterCloud to receive the filtered cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessor.FilterCloud(inputCloud, 0.2, Eigen::Vector4f (-20, -5, -2.5, 1), Eigen::Vector4f (20, 7, 5, 1));
    
    // Segmentation process
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloudI = pointProcessor.SegmentPlane (filterCloud, 100, 0.25);

    // Clustering process
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClustersI = pointProcessor.Clustering(segmentCloudI.first, 1.0, 10, 600);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color (1,1,0), Color(0,0,1)};

    // Rendering
    // Render segmentated cloud
    if(render_obst)
        renderPointCloud(viewer,segmentCloudI.first,"obstCloud",Color(1,0,0));
    if(render_plane)
        renderPointCloud(viewer,segmentCloudI.second,"planeCloud",Color(0,1,0));
    // Render filtered cloud
    if (filteredCloud)
        renderPointCloud(viewer,filterCloud,"filterCloud");

    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClustersI)
    {
        if(render_clusters)
        {
            std::cout << "cluster size ";
            pointProcessor.numPoints(cluster);
            renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        }
        
        if (render_box)
        {
            Box box = pointProcessor.BoundingBox(cluster);
            renderBox(viewer,box,clusterId);
        }

       
        ++clusterId;
    }

}

//void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI> pointProcessor, pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud)
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block ---------
    // ----------------------------------------------------

    // 1 will be rendered, 0 will not
    int render_obst = 1;
    int render_plane = 1;
    int filteredCloud = 0;
    int render_clusters = 0;
    int render_box = 1;
    int render_ransac1 = 1;
    int render_ransac2 = 1;

    // Create pointProcessor
    ProcessPointClouds<pcl::PointXYZI> pointProcessor;
    // Create pointProcessorI
    //ProcessPointClouds<pcl::PointXYZI> pointProcessorI;

    // Load data into inputCloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessor.loadPcd("/home/workspace/SFND_Lidar_Obstacle_Detection/src/sensors/data/pcd/data_1/0000000000.pcd");
    // Render cloud
    //renderPointCloud(viewer,inputCloud,"cloud");

    // Create filterCloud to receive the filtered cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessor.FilterCloud(inputCloud, 0.2, Eigen::Vector4f (-20, -5, -2.5, 1), Eigen::Vector4f (20, 7, 5, 1));
    // Render cloud
    //renderPointCloud(viewer,filterCloud,"cloud");
    
    /*
    // Segmentation process
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloudI = pointProcessor.SegmentPlane (filterCloud, 100, 0.25);
    // Render cloud
    renderPointCloud(viewer,segmentCloudI.first,"obstCloud",Color(1,0,0));
    renderPointCloud(viewer,segmentCloudI.second,"planeCloud",Color(0,1,0));
    */
    
    //std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr,pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = Ransac3D(filterCloud,100,0.5);

    // Segmentation process using Ransac 3D
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloudI = pointProcessor.Ransac3D(filterCloud, 100, 0.25);

    if(render_ransac1)
        renderPointCloud(viewer,segmentCloudI.first,"obstCloud",Color(1,0,0));
    if(render_ransac2)
        renderPointCloud(viewer,segmentCloudI.second,"planeCloud",Color(0,1,0));

    /*
    // Clustering process
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClustersI = pointProcessor.Clustering(segmentCloudI.first, 1.0, 10, 600);

    //KdTree* tree = new Kdtree;

    //for (int i=0; i<segmentCloudI.size(); i++)
    //    tree->insert(segmentCloudI[i],i);

    // Euclidean clustering
    //std::vector<std::vector<int>> cloudClustersI = euclideanClustering(segmentCloudI.first, tree, 3.0);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color (1,1,0), Color(0,0,1)};

    // Rendering
    // Render segmentated cloud
    if(render_obst)
        renderPointCloud(viewer,segmentCloudI.first,"obstCloud",Color(1,0,0));
    if(render_plane)
        renderPointCloud(viewer,segmentCloudI.second,"planeCloud",Color(0,1,0));
    // Render filtered cloud
    if (filteredCloud)
        renderPointCloud(viewer,filterCloud,"filterCloud");

    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClustersI)
    {
        if(render_clusters)
        {
            std::cout << "cluster size ";
            pointProcessor.numPoints(cluster);
            renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        }
        
        if (render_box)
        {
            Box box = pointProcessor.BoundingBox(cluster);
            renderBox(viewer,box,clusterId);
        }

       
        ++clusterId;
    }
    */
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}

int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);
    cityBlock(viewer);

    /*
    ProcessPointClouds<pcl::PointXYZI> pointProcessorI;

    std::vector<boost::filesystem::path> stream = pointProcessorI.streamPcd("/home/workspace/SFND_Lidar_Obstacle_Detection/src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();

    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
    
    while (!viewer->wasStopped ())
    {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        inputCloudI = pointProcessorI.loadPcd((*streamIterator).string());
        builtIncityBlock(viewer, pointProcessorI, inputCloudI);
        //cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
    } 
    */
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 
  
}