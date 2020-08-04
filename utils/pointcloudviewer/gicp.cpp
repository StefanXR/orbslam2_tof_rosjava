#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/gicp.h>

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr align(pcl::Registration<pcl::PointXYZRGBA, pcl::PointXYZRGBA>::Ptr registration, const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& target_cloud, const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& source_cloud ) {

  registration->setInputTarget(target_cloud);
  registration->setInputSource(source_cloud);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZRGBA>());
  registration->align(*aligned);

  return aligned;
}


int main(int argc, char** argv) {

  //INPUT FILES
  if(argc != 3) {
    std::cout << "usage: align target.pcd source.pcd" << std::endl;
    return 0;
  }

  std::string target_pcd = argv[1];
  std::string source_pcd = argv[2];

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());

  if(pcl::io::loadPCDFile(target_pcd, *target_cloud)) {
    std::cerr << "failed to load " << target_pcd << std::endl;
    return 0;
  }
  if(pcl::io::loadPCDFile(source_pcd, *source_cloud)) {
    std::cerr << "failed to load " << source_pcd << std::endl;
    return 0;
  }

  // downsampling
  /*pcl::PointCloud<pcl::PointXYZRGBA>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZRGBA>());

  pcl::VoxelGrid<pcl::PointXYZRGBA> voxelgrid;
  voxelgrid.setLeafSize(0.1f, 0.1f, 0.1f);

  voxelgrid.setInputCloud(target_cloud);
  voxelgrid.filter(*downsampled);
  *target_cloud = *downsampled;

  voxelgrid.setInputCloud(source_cloud);
  voxelgrid.filter(*downsampled);
  source_cloud = downsampled;*/

  std::cout << "--- pcl::GICP ---" << std::endl;
  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA>::Ptr gicp(new pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA>());
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr aligned = align(gicp, target_cloud, source_cloud);

  // visulization
  pcl::visualization::PCLVisualizer vis("vis");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> target_handler(target_cloud, 255.0, 0.0, 0.0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> source_handler(source_cloud, 0.0, 255.0, 0.0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> aligned_handler(aligned, 0.0, 0.0, 255.0);
  vis.setBackgroundColor (1, 1, 1);
  //vis.addCoordinateSystem (1.0);
  vis.addPointCloud(target_cloud, target_handler, "target");
  vis.addPointCloud(source_cloud, source_handler, "source");
  vis.addPointCloud(aligned, aligned_handler, "aligned");
  vis.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "target");
  vis.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "source");
  vis.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "aligned");
  vis.spin();

  return 0;
}
