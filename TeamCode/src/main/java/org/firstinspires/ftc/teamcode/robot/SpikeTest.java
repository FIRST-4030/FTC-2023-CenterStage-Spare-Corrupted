package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.drives.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.general.Pose2dWrapper;
import org.firstinspires.ftc.teamcode.drive.drives.CustomMecanumDrive;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.Endpoint;

import java.util.ArrayList;

@Config
@Autonomous(name = "SpikeTest")
public class SpikeTest extends LinearOpMode {
    public static boolean BLUE = false;
    public static int SPIKE = 2;
    public static double SPIKE_POINT_X = 12;
    public static double SPIKE_POINT_Y = -33;
    public static double HEADING = 90;
    public static double POINTX = 45;
    public static double POINTY = -60;
    public static double POINTHEADING = 90;
    public static boolean LEFT = false;
    public ArrayList<Endpoint> endpoints = new ArrayList<Endpoint>();

    public Pose2dWrapper startPose = new Pose2dWrapper(15, -62, Math.toRadians(90));
    public Pose2dWrapper resetPose = new Pose2dWrapper(13, -50, -90);
    public Pose2dWrapper mediaryPose = new Pose2dWrapper(36.5, -60, -90);
    public Pose2dWrapper backdropPose = new Pose2dWrapper(POINTX, -36, 0);
    public Pose2dWrapper centerPose = new Pose2dWrapper(36.5, -12, 90);
    public Pose2dWrapper avoidancePose = new Pose2dWrapper(30.5 , -12, 90);
    //X values get wonky here, as invertLeft is ran on all Endpoints used on the left starting point,
    //so numbers are less than would be expected and sometimes greater than 70, however invertLeft() clears this up,
    //decided to do this for readability in the if(LEFT) statement
    public Pose2dWrapper travelPose = new Pose2dWrapper(-54, -12, 0);

    public ComputerVision vision = new ComputerVision(hardwareMap);




    @Override
    public void runOpMode() throws InterruptedException {
        CustomMecanumDrive drive = new CustomMecanumDrive(hardwareMap, 1, 1, 1);

        switch(SPIKE){
            case 1:
                SPIKE_POINT_X = 6;
                SPIKE_POINT_Y = -37;
                HEADING = 135;
                break;
            case 2:
                SPIKE_POINT_X = 12;
                SPIKE_POINT_Y = -33;
                HEADING = 90;
                break;
            case 3:
                SPIKE_POINT_X = 18;
                SPIKE_POINT_Y = -40;
                HEADING = 45;
                break;
        }

        Endpoint spikePoint = new Endpoint(SPIKE_POINT_X, SPIKE_POINT_Y, HEADING);
        Endpoint resetPoint = new Endpoint(resetPose.x, resetPose.y, resetPose.heading);
        Endpoint mediaryPoint = new Endpoint(mediaryPose.x, mediaryPose.y, mediaryPose.heading);
        Endpoint backdropPoint = new Endpoint(backdropPose.x, backdropPose.y, backdropPose.heading);
        Endpoint centerPoint = new Endpoint(centerPose.x, centerPose.y, centerPose.heading);
        Endpoint avoidancePoint = new Endpoint(avoidancePose.x, avoidancePose.y, avoidancePose.heading);
        Endpoint travelPoint = new Endpoint(travelPose.x, travelPose.y, travelPose.heading);
        endpoints.add(spikePoint);
        endpoints.add(resetPoint);
        endpoints.add(mediaryPoint);
        endpoints.add(backdropPoint);
        endpoints.add(centerPoint);
        endpoints.add(avoidancePoint);
        endpoints.add(travelPoint);

        if(LEFT){
            startPose.x += 24;
            startPose.x *= -1;

            //Change heading value to match mirrored spikes
            if (HEADING == 135){
                spikePoint.setHeading(45);
            } else if (HEADING == 45){
                spikePoint.setHeading(135);
            }

            spikePoint.invertLeft();
            resetPoint.invertLeft();
            mediaryPoint.invertLeft();
            centerPoint.invertLeft();
            avoidancePoint.invertLeft();
            travelPoint.invertLeft();
        }
        if(BLUE){
            startPose.y *= -1;
            startPose.heading *= -1;
            spikePoint.invertSides();
            resetPoint.invertSides();
            mediaryPoint.invertSides();
            backdropPoint.invertSides();
            centerPoint.invertSides();
            avoidancePoint.invertSides();
            travelPoint.invertSides();
        }


        drive.setPoseEstimate(startPose.toPose2d());

        Trajectory spikeTraj = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .splineTo(spikePoint.getPos(), Math.toRadians(spikePoint.getHeading()))
                                .build();
        Trajectory resetTraj = drive.trajectoryBuilder(spikeTraj.end(), true)
                        .splineTo(resetPoint.getPos(), Math.toRadians(resetPoint.getHeading()))
                                .build();
        Trajectory mediaryTraj = drive.trajectoryBuilder(resetTraj.end())
                        .strafeTo(mediaryPoint.getPos())
                                .build();
        Trajectory backdropTraj = drive.trajectoryBuilder(mediaryTraj.end())
                .splineTo(backdropPoint.getPos(), Math.toRadians(backdropPoint.getHeading()))
                .build();
        Trajectory combinedTraj = drive.trajectoryBuilder(spikeTraj.end(), true)
                .splineTo(resetPoint.getPos(), Math.toRadians(resetPoint.getHeading()))
                .splineTo(mediaryPoint.getPos(), Math.toRadians(mediaryPoint.getHeading()))
                .build();

        Trajectory centerTraj = drive.trajectoryBuilder(combinedTraj.end())
                        .splineTo(centerPoint.getPos(), Math.toRadians(centerPoint.getHeading()))
                                .build();
        Trajectory travelTraj = drive.trajectoryBuilder(centerTraj.end())
                        .lineToLinearHeading(travelPoint.getPose())
                                .build();
        Trajectory leftBackdropTraj = drive.trajectoryBuilder(travelTraj.end())
                .strafeTo(backdropPoint.getPos())
                .build();





        if(LEFT){
            DashboardUtil.previewTrajectories(FtcDashboard.getInstance(), spikeTraj, combinedTraj, centerTraj, travelTraj, leftBackdropTraj);
        } else {
            DashboardUtil.previewTrajectories(FtcDashboard.getInstance(), spikeTraj, combinedTraj, backdropTraj);
        }

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(spikeTraj);
        drive.followTrajectory(combinedTraj);
        if(!LEFT) {
            drive.followTrajectory(backdropTraj);
        }
        if(LEFT){
            drive.followTrajectory(centerTraj);
            drive.followTrajectory(travelTraj);
            drive.followTrajectory(leftBackdropTraj);
        }
    }
}
