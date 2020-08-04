/*
 * PointCloudMapping.h
 * 
 * Dense point cloud extension for the ORB SLAM 2 algorithm.
 * 
 * for the bachelor thesis by Stefan Bauer, 2020
 * 
 */

#ifndef POINTCLOUDMAPPING_H
#define POINTCLOUDMAPPING_H

#include "System.h"
#include <KeyFrame.h>
#include "Converter.h"

#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/gicp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>

#include <opencv2/highgui/highgui.hpp>
#include <condition_variable>

using namespace ORB_SLAM2;

class PointCloudMapping
{
public:
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
    
    PointCloudMapping( double resolution_ );
    
    // for every keyframe the map is updated.
    void insertKeyFrame( KeyFrame* kf, cv::Mat& color, cv::Mat& depth );
    void shutdown();
    void viewer();
    
protected:
    PointCloud::Ptr generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr align(pcl::Registration<pcl::PointXYZRGBA, pcl::PointXYZRGBA>::Ptr registration, const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& target_cloud, const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& source_cloud );

    
    PointCloud::Ptr globalMap; //blue
    PointCloud::Ptr localMap; //red

    shared_ptr<thread>  viewerThread;   
    
    bool    shutDownFlag    =false;
    mutex   shutDownMutex;  
    
    condition_variable  keyFrameUpdated;
    mutex               keyFrameUpdateMutex;
    
    // data to generate point clouds
    vector<KeyFrame*>       keyframes;
    vector<cv::Mat>         colorImgs;
    vector<cv::Mat>         depthImgs;
    mutex                   keyframeMutex;
    uint16_t                lastKeyframeSize =0;
    
    //resolution for the pointcloud
    double resolution = 0.02;
    pcl::VoxelGrid<PointT>  voxel;

};
#endif // POINTCLOUDMAPPING_H