package com.github.rosjava.orbslam2_tof_java.roslistener;

import java.util.ArrayList;

//Datatype for depth image
public class ImageRaw{

    private int height;
    private int width;
    private ArrayList<Byte> mat;
    private int secs;
    private int nsecs;
    public int getHeight(){
        return this.height;
    };



    public int getWidth(){
        return this.width;
    };

    public int getSecs(){
        return this.secs;
    };

    public int getNsecs(){
        return this.nsecs;
    };

    public ArrayList<Byte> getMat(){
        return this.mat;
    };



    public void setHeight(int height){
        this.height = height;
    };

    public void setWidth(int width){
        this.width = width;
    };

    public void setMat(ArrayList<Byte> mat){
        this.mat = mat;
    };

    public void setSecs(int secs){
        this.secs = secs;
    };

    public void setNsecs(int nsecs){
        this.nsecs = nsecs;
    };

}