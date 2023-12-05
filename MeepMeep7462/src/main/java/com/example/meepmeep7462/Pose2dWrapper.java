package com.example.meepmeep7462;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class Pose2dWrapper {

    public double x, y, heading;

    public Pose2dWrapper(double x, double y, double heading){
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public Pose2d toPose2d(){
        return new Pose2d(x, y, heading);
    }
}
