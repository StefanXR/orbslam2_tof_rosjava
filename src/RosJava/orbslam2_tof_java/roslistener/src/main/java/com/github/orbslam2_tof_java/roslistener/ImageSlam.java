package com.github.rosjava.orbslam2_tof_java.roslistener;

import java.util.ArrayList;

//Datatype for depth image with position and orientation from camera 
public class ImageSlam{

    public double pointX;
    public double pointY;
    public double pointZ;

    public double quaternionX;
    public double quaternionY;
    public double quaternionZ;
    public double quaternionW;    
 
    private ImageRaw imageRaw;

    private int secs;
    private int nsecs;


    public int getSecs(){
        return this.secs;
    };

    public int getNsecs(){
        return this.nsecs;
    };

    public void setSecs(int secs){
        this.secs = secs;
    };

    public void setNsecs(int nsecs){
        this.nsecs = nsecs;
    };

    public void setImageRaw(ImageRaw imageraw){
        this.imageRaw = imageRaw;
    }

    public ImageRaw getImageRaw(ImageRaw imageraw){
        return this.imageRaw;
    }


}
