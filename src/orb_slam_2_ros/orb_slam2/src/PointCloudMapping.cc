/*
 * PointCloudMapping.cpp
 * 
 * Dense point cloud extension for the ORB SLAM 2 algorithm.
 * 
 * for the bachelor thesis by Stefan Bauer, 2020
 * 
 */

#include "PointCloudMapping.h"
#include <iostream>

PointCloudMapping::PointCloudMapping(double resolution_)
{
    voxel.setLeafSize( resolution, resolution, resolution);
    globalMap = boost::make_shared<PointCloud>();
    localMap = boost::make_shared<PointCloud>();
    
    viewerThread = make_shared<thread>( bind(&PointCloudMapping::viewer, this ) );
}

void PointCloudMapping::shutdown()
{
    {
        unique_lock<mutex> lck(shutDownMutex);
        shutDownFlag = true;
        keyFrameUpdated.notify_one();
    }
    viewerThread->join();
}

void PointCloudMapping::insertKeyFrame(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
{
    cout<<"receive a keyframe, id = "<<kf->mnId<<endl;
    unique_lock<mutex> lck(keyframeMutex);
    keyframes.push_back( kf );
    colorImgs.push_back( color.clone() );
    depthImgs.push_back( depth.clone() );
    
    keyFrameUpdated.notify_one();
}

pcl::PointCloud< PointCloudMapping::PointT >::Ptr PointCloudMapping::generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
{
    PointCloud::Ptr tmp( new PointCloud() );

    //set points with depth image in the cloud for every keyframe
    if(depth.empty() == false){

        for ( int m=0; m < depth.rows; m++ )
        {
            for ( int n=0; n<depth.cols; n++ )
            {
                float d = depth.ptr<float>(m)[n];
                if (d < 0.01 || d>10)
                    continue;
                PointT p;

                //assign every point with keyframe position
                p.z = d;
                p.x = ((n - kf->cx) * p.z / kf->fx);
                p.y = ((m - kf->cy) * p.z / kf->fy);
           
                //set the "color" information for every voxel 
                p.b = color.ptr<uchar>(m)[n*3];
                p.g = color.ptr<uchar>(m)[n*3+1];
                p.r = color.ptr<uchar>(m)[n*3+2];
                    
                tmp->points.push_back(p);
            }
        }
    }
    
    Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat( kf->GetPose() );

    PointCloud::Ptr cloud(new PointCloud);
    pcl::transformPointCloud( *tmp, *cloud, T.inverse().matrix());
    cloud->is_dense = false;

    //rotate the pointcloud 180° along the Z-axis and Y-axis
    Eigen::Affine3f transformZ = Eigen::Affine3f::Identity();
    transformZ.rotate (Eigen::AngleAxisf (3.141593, Eigen::Vector3f::UnitZ()));
    pcl::transformPointCloud (*cloud, *cloud, transformZ);
    Eigen::Affine3f transformY = Eigen::Affine3f::Identity();
    transformY.rotate (Eigen::AngleAxisf (3.141593, Eigen::Vector3f::UnitY()));
    pcl::transformPointCloud (*cloud, *cloud, transformY);

    return cloud;
}

void PointCloudMapping::viewer()
{
    int counter = 0;
    pcl::visualization::PCLVisualizer vis("vis");
    vis.setBackgroundColor (1, 1, 1);

    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA>::Ptr gicp(new pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA>());
    PointCloud::Ptr emptyCloud(new PointCloud());

    while(true)
    {
        {
            unique_lock<mutex> lck_shutdown( shutDownMutex );
            if (shutDownFlag)
            {
                break;
            }
        }

        {
            unique_lock<mutex> lck_keyframeUpdated( keyFrameUpdateMutex );
            keyFrameUpdated.wait( lck_keyframeUpdated );
        }
        
        // keyframe is updated 
        size_t N=0;
        {
            unique_lock<mutex> lck( keyframeMutex );
            N = keyframes.size();
        }
        
        for ( size_t i=lastKeyframeSize; i<N ; i++ )
        {
            PointCloud::Ptr p = generatePointCloud( keyframes[i], colorImgs[i], depthImgs[i] );

            *localMap += *p;
        }

        PointCloud::Ptr tmp(new PointCloud());
        voxel.setInputCloud( localMap );
        voxel.filter( *tmp );

        localMap->swap( *tmp );
        
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> localMap_handler(localMap, 255.0, 0.0, 0.0); //red
 
        counter++;

        //add pointclouds to the system 
        if(!vis.contains("local") && !vis.contains("global"))
        { 
            vis.addPointCloud(localMap, localMap_handler, "local");
            vis.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "local");
        }
        else if(vis.contains("local") && !vis.contains("global"))
        {

            vis.updatePointCloud(localMap, localMap_handler,"local");

            if(counter == 2)
            {
                counter = 0;
                globalMap->swap ( *localMap);
                vis.addPointCloud(globalMap, "global");
                vis.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "global");
                *localMap = *emptyCloud;
            }

        }
        else if(vis.contains("local") && vis.contains("global"))
        {
            
            vis.updatePointCloud(localMap, localMap_handler,"local");

            if(counter == 2) 
            {
                counter = 0;
                //globalMap  = align(gicp,  globalMap, localMap);  //align every to keyframes the globalMap and the localMap
                *globalMap += *localMap;
                vis.updatePointCloud(globalMap, "global");
                //pcl::io::savePCDFile ("~path/globalmap.pcd"savePath, *globalMap); //uncomment if the pointcloud should be safed
                *localMap = *emptyCloud;
            }
        }

        vis.spinOnce(100, false);
        lastKeyframeSize = N;
    }
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PointCloudMapping::align(pcl::Registration<pcl::PointXYZRGBA, pcl::PointXYZRGBA>::Ptr registration, const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& target_cloud, const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& source_cloud ) 
{

  registration->setInputTarget(target_cloud);
  registration->setInputSource(source_cloud);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZRGBA>());
  registration->align(*aligned);

  return aligned;
}
