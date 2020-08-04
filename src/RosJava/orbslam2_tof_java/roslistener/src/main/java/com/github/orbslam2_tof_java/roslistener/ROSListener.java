package com.github.rosjava.orbslam2_tof_java.roslistener;

import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Subscriber;

import java.util.ArrayList;

public class ROSListener extends AbstractNodeMain {

  ArrayList<ImageRaw> imageRawArray = new ArrayList<ImageRaw>();
  ArrayList<ImageSlam> imageSlamArray = new ArrayList<ImageSlam>();
  
  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("rosjava/roslistener");
  }

  @Override
  public void onStart(ConnectedNode connectedNode) {
    

    Subscriber<sensor_msgs.Image> depthImage = connectedNode.newSubscriber("pico_flexx/image_depth", sensor_msgs.Image._TYPE);
    depthImage.addMessageListener(new MessageListener<sensor_msgs.Image>() {
    @Override
    public void onNewMessage(sensor_msgs.Image message) {
        ImageRaw imageRaw = new ImageRaw();

        imageRaw.setHeight(message.getHeight());
        imageRaw.setWidth(message.getWidth());
        imageRaw.setSecs(message.getHeader().getStamp().secs);
        imageRaw.setNsecs(message.getHeader().getStamp().nsecs);

        ArrayList<Byte> mat = new ArrayList<Byte>();

        for(int row = 0; row < imageRaw.getHeight() ; row++) { 
          for(int column = 0; column < imageRaw.getWidth(); column++) {

              mat.add(message.getData().getByte(row*imageRaw.getHeight()+column));
            }
        }
        imageRaw.setMat(mat);
        imageRawArray.add(imageRaw);

      }
    });
 
    
    Subscriber<geometry_msgs.PoseStamped> Pose = connectedNode.newSubscriber("orb_slam2_tof/pose", geometry_msgs.PoseStamped._TYPE);
    Pose.addMessageListener(new MessageListener<geometry_msgs.PoseStamped>() {
          @Override
          public void onNewMessage(geometry_msgs.PoseStamped poseMessage) {
            ImageSlam imageSlam = new ImageSlam();

            imageSlam.setSecs(poseMessage.getHeader().getStamp().secs);
            imageSlam.setNsecs(poseMessage.getHeader().getStamp().nsecs);

            imageSlam.pointX = poseMessage.getPose().getPosition().getX();
            imageSlam.pointY = poseMessage.getPose().getPosition().getY();
            imageSlam.pointZ = poseMessage.getPose().getPosition().getZ();

            imageSlam.quaternionX = poseMessage.getPose().getOrientation().getX();
            imageSlam.quaternionY = poseMessage.getPose().getOrientation().getY();
            imageSlam.quaternionZ = poseMessage.getPose().getOrientation().getZ();          
            imageSlam.quaternionW = poseMessage.getPose().getOrientation().getW();  

            while(imageRawArray.size() < 0){
              for(int i=imageRawArray.size()-1 ; i>=0; i--){
               
                if((imageRawArray.get(i).getSecs() + imageRawArray.get(i).getNsecs()) == (imageSlam.getSecs()+imageSlam.getNsecs()))
                {
                 imageSlam.setImageRaw(imageRawArray.get(i));
                 i=0;
                }
              }
            }

            imageSlamArray.add(imageSlam);
            //TODO Convert image format to bugbot image format
          }
    });
  }

  public ImageRaw getLastimageRaw (){

    // TODO: Code for communication between rosjava and bugbot
    return imageRawArray.get(imageRawArray.size()-1);
  }

  public ImageSlam getLastimageSlam (){

    // TODO: Code for communication between rosjava and bugbot
    return imageSlamArray.get(imageSlamArray.size()-1);
  }

  //TODO: Add Pointcloud

}

