package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;

import org.firstinspires.ftc.teamcode.drive.DriveConstants2;
import org.firstinspires.ftc.teamcode.general.Pose2dWrapper;

public class Endpoint2 {

    public Pose2dWrapper pose;
    public double velocityConstraint, accelerationConstraint;

    public Endpoint2(){
        this.pose = new Pose2dWrapper(0, 0, 0);
        this.velocityConstraint = DriveConstants2.MAX_VEL;
        this.accelerationConstraint = DriveConstants2.MAX_ACCEL;
    }

    public Endpoint2(double posX, double posY, double heading){
        this.pose = new Pose2dWrapper(posX, posY, heading);
        this.velocityConstraint = DriveConstants2.MAX_VEL;
        this.accelerationConstraint = DriveConstants2.MAX_ACCEL;
    }

    public Endpoint2(double posX, double posY, double heading, double velocityConstraint, double accelerationConstraint){
        this.pose = new Pose2dWrapper(posX, posY, heading);
        this.velocityConstraint = velocityConstraint;
        this.accelerationConstraint = accelerationConstraint;
    }

    public Vector2d getPos(){
        return new Vector2d(pose.x, pose.y);
    }

    public void setHeading(double newHeading){
        pose.heading = newHeading;
    }

    public double getHeading(){
        return pose.heading;
    }


    public Vector2d getDir(){
        return pose.toPose2d().headingVec();
    }

    public Pose2d getPose(){
        return pose.toPose2d();
    }

    public MecanumVelocityConstraint getMaxVel(){
        return new MecanumVelocityConstraint(velocityConstraint, DriveConstants2.TRACK_WIDTH);
    }

    public ProfileAccelerationConstraint getMaxAccel(){
        return new ProfileAccelerationConstraint(accelerationConstraint);
    }

    public void invertSides(){
        pose.y = -pose.y;
        pose.heading = -getHeading();
    }
    public void invertLeft(){
        pose.x = -(pose.x + 34);
    }
    public void invertSpikeLeft(){
        pose.x = pose.x - 47;
    }
}
