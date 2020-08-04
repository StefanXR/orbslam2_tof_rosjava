#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
 
int main(int argc, char** argv){
  
  //INPUT FILES
  if(argc != 2) {
    std::cout << "usage: viewer cloud.pcd" << std::endl;
    return 0;
  }
  
  std::string cloud_pcd = argv[1];

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

  if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (cloud_pcd, *cloud) == -1){
    std::cout << "Couldn't read file" << cloud_pcd << std::endl;
    return (-1);
  }
  std::cout << "Loaded ..." << std::endl;

  //pcl::visualization::CloudViewer viewer("Cloud Viewer"); //color
  pcl::visualization::PCLVisualizer viewer("Cloud Viewer"); //blue
  viewer.setBackgroundColor(1, 1, 1);
  
  //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> cloud_handler(cloud, 0.0, 0.0, 255.0); //blue

  //viewer.addPointCloud(cloud, cloud_handler, "cloud"); //blue
  viewer.addPointCloud(cloud, "cloud"); //color
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.3, "cloud");

  //viewer.runOnVisualizationThread (viewerPsycho);
  while (!viewer.wasStopped ())
  {
    viewer.spin();
  }
  return 0;
}
