#include "MonoToFNode.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "MonoToF");
    ros::start();

    if(argc > 1) {
        ROS_WARN ("Arguments supplied via command line are neglected.");
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ros::NodeHandle node_handle;
    image_transport::ImageTransport image_transport (node_handle);

    MonoToFNode node (ORB_SLAM2::System::MONOTOF, node_handle, image_transport);

    node.Init();

    ros::spin();

    ros::shutdown();

    return 0;
}



MonoToFNode::MonoToFNode (const ORB_SLAM2::System::eSensor sensor, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport) : Node (sensor, node_handle, image_transport) {
  rgb_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, "/pico_flexx/image_mono8", 1);
  depth_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, "/pico_flexx/image_depth", 1);
  camera_info_topic_ = "/pico_flexx/camera_info";

  sync_ = new message_filters::Synchronizer<sync_pol> (sync_pol(10), *rgb_subscriber_, *depth_subscriber_);
  sync_->registerCallback(boost::bind(&MonoToFNode::ImageCallback, this, _1, _2));
}

MonoToFNode::~MonoToFNode () {
  delete rgb_subscriber_;
  delete depth_subscriber_;
  delete sync_;
}



void MonoToFNode::ImageCallback (const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD) {
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptrRGB;
  try {
      cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  cv_bridge::CvImageConstPtr cv_ptrD;
  try {
    cv_ptrD = cv_bridge::toCvShare(msgD);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

    cv::Mat zImage = cv_ptrD->image;
    cv::Mat monoImage = cv_ptrRGB->image;

    //monoImage.create (cv::Size (im.cols, im.rows), CV_8UC1);
    //zImage.create (cv::Size (depthmap.cols, depthmap.rows), CV_32FC1);

    // std::cout << "---\nMonoImage Type:" << monoImage.type() << endl;
    // std::cout << "MonoImage Width:" << monoImage.cols << " Height: " << monoImage.rows << endl;

    // std::cout << "DepthImage Type:" << zImage.type() << endl;
    // std::cout << "DepthImage Width:" << zImage.cols << " Height: " << zImage.rows << endl;

    // im.convertTo (rgbImage, CV_8UC3);
    // depthmap.convertTo (zImage, CV_32FC1);

    // scaledRgbImage.create (cv::Size(640,480), CV_8UC3);
    // scaledZImage.create (cv::Size(640,480), CV_32FC1);


    cv::resize (monoImage, monoImage, cv::Size(640,480));
    cv::resize (zImage, zImage, cv::Size(640,480));

    cv::namedWindow("Depth");
    cv::moveWindow("Depth", 400,300);
    cv::imshow("Depth", zImage); 

    cv::namedWindow("Mono");
    cv::moveWindow("Mono", 1100,300);
    cv::imshow("Mono", monoImage);

    cv::waitKey(10);

  current_frame_time_ = msgRGB->header.stamp;

  orb_slam_->TrackMonoToF(monoImage,zImage,cv_ptrRGB->header.stamp.toSec());

  Update ();

}