package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.drives.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.gamepad.gamepad.InputAutoMapper;
import org.firstinspires.ftc.teamcode.gamepad.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.general.Pose2dWrapper;
import org.firstinspires.ftc.teamcode.drive.drives.CustomMecanumDrive;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.Endpoint;

import java.util.ArrayList;

@Config
@Autonomous(name = "SpikeTest")
public class SpikeTest extends LinearOpMode {
    public static boolean BLUE = false;
    public static int SPIKE = 0;
    public int spike = 0;
    public static double SPIKE_POINT_X = 12;
    public static double SPIKE_POINT_Y = -33;
    public static double HEADING = 90;
    public static double BACKDROP_Y = -35;
    public static double POINTX = 45;
    public static double POINTY = -60;
    public static double POINTHEADING = 90;
    public static boolean LEFT = false;
    public ArrayList<Endpoint> endpoints = new ArrayList<Endpoint>();

    public Pose2dWrapper startPose = new Pose2dWrapper(15, -61, Math.toRadians(90));
    public Pose2dWrapper resetPose = new Pose2dWrapper(13, -50, -90);
    public Pose2dWrapper mediaryPose = new Pose2dWrapper(15, -40, 0);
    public Pose2dWrapper audiencePose = new Pose2dWrapper(-58, -40, 0);
    public Pose2dWrapper backdropPose = new Pose2dWrapper(48, BACKDROP_Y, 0);
    public Pose2dWrapper centerPose = new Pose2dWrapper(-58, -12, 0);
    public Pose2dWrapper avoidancePose = new Pose2dWrapper(30.5 , -12, 90);
    public Pose2dWrapper tempParkPose = new Pose2dWrapper(54, -12, 0);
    //X values get wonky here, as invertLeft is ran on all Endpoints used on the left starting point,
    //so numbers are less than would be expected and sometimes greater than 70, however invertLeft() clears this up,
    //decided to do this for readability in the if(LEFT) statement
    public Pose2dWrapper travelPose = new Pose2dWrapper(35, -12, 0);

    ComputerVision vision;
    InputHandler inputHandler;
    Servo armServo;

    ElapsedTime operationTimer;
    boolean inputComplete = false;
    boolean isBlue = false;




    @Override
    public void runOpMode() throws InterruptedException {
        inputHandler = InputAutoMapper.normal.autoMap(this);
        while(inputComplete = false){
            inputHandler.loop();
            if(inputHandler.up("D1:DPAD_LEFT")){
                isBlue = !isBlue;
            }
            if(inputHandler.up("D1:DPAD_RIGHT")){
                LEFT = !LEFT;
            }
            if(inputHandler.up("D1:X")){
                inputComplete = true;
            }
            telemetry.addData("is Blue:", isBlue);
            telemetry.addData("is Left:", LEFT);
            telemetry.addData("Press X to finalize values", "");
        }
        NewMecanumDrive drive = new NewMecanumDrive(hardwareMap);
        vision = new ComputerVision(hardwareMap);
        armServo = hardwareMap.get(Servo.class, "Arm");
        operationTimer = new ElapsedTime();

        while(spike == 0) {
            vision.update();
            spike = vision.checkSpike(isBlue);
            sleep(1);
        }

        switch (spike) {
            case 1:
                SPIKE_POINT_X = 8;
                SPIKE_POINT_Y = -35;
                HEADING = 145;
                BACKDROP_Y = -30;
                break;
            case 2:
                SPIKE_POINT_X = 11;
                SPIKE_POINT_Y = -34;
                HEADING = 90;
                BACKDROP_Y = -35;
                break;
            case 3:
                SPIKE_POINT_X = 17;
                SPIKE_POINT_Y = -34;
                HEADING = 55;
                BACKDROP_Y = -40;
                break;
        }



        Endpoint spikePoint = new Endpoint(SPIKE_POINT_X, SPIKE_POINT_Y, HEADING);
        Endpoint resetPoint = new Endpoint(resetPose.x, resetPose.y, resetPose.heading);
        Endpoint mediaryPoint = new Endpoint(mediaryPose.x, mediaryPose.y, mediaryPose.heading);
        Endpoint backdropPoint = new Endpoint(backdropPose.x, backdropPose.y, backdropPose.heading);
        Endpoint centerPoint = new Endpoint(centerPose.x, centerPose.y, centerPose.heading);
        Endpoint avoidancePoint = new Endpoint(avoidancePose.x, avoidancePose.y, avoidancePose.heading);
        Endpoint travelPoint = new Endpoint(travelPose.x, travelPose.y, travelPose.heading);
        Endpoint audiencePoint = new Endpoint(audiencePose.x, audiencePose.y, audiencePose.heading);
        Endpoint tempParkPoint = new Endpoint(tempParkPose.x, tempParkPose.y, tempParkPose.heading);
        endpoints.add(spikePoint);
        endpoints.add(resetPoint);
        endpoints.add(mediaryPoint);
        endpoints.add(backdropPoint);
        endpoints.add(centerPoint);
        endpoints.add(avoidancePoint);
        endpoints.add(travelPoint);
        endpoints.add(audiencePoint);

        if(LEFT){
            startPose.x += 24;
            startPose.x *= -1;

            //Change heading value to match mirrored spikes
            /*if (HEADING == 145){
                spikePoint.setHeading(55);
            } else if (HEADING == 55){
                spikePoint.setHeading(145);
            }
             */

            spikePoint.invertSpikeLeft();
            resetPoint.invertLeft();
            mediaryPoint.invertLeft();
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
            audiencePoint.invertSides();
            tempParkPoint.invertSides();
        }


        drive.setPoseEstimate(startPose.toPose2d());

        Trajectory spikeTraj = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .splineTo(spikePoint.getPos(), Math.toRadians(spikePoint.getHeading()))
                                .build();
        /*Trajectory resetTraj = drive.trajectoryBuilder(spikeTraj.end(), true)
                        .splineTo(resetPoint.getPos(), Math.toRadians(resetPoint.getHeading()))
                                .build();
         */
        Trajectory mediaryTraj = drive.trajectoryBuilder(spikeTraj.end())
                        .strafeTo(mediaryPoint.getPos())
                                .build();
        Trajectory backdropTraj = drive.trajectoryBuilder(mediaryTraj.end())
                .splineTo(backdropPoint.getPos(), Math.toRadians(backdropPoint.getHeading()))
                .build();

        //modify to avoid wing and end with a heading of 0 and target further towards center
        /*Trajectory combinedTraj = drive.trajectoryBuilder(spikeTraj.end(), true)
                .splineTo(resetPoint.getPos(), Math.toRadians(resetPoint.getHeading()))
                .splineTo(mediaryPoint.getPos(), Math.toRadians(mediaryPoint.getHeading()))
                .build();
         */

        //modify to strafe w/ heading of 0
        Trajectory centerTraj = drive.trajectoryBuilder(mediaryTraj.end())
                        .splineTo(centerPoint.getPos(), Math.toRadians(centerPoint.getHeading()))
                                .build();
        //reverse to pixels
        //intake pixel (turn intake on, use flippers)
        //disable intake

        Trajectory travelTraj = drive.trajectoryBuilder(centerTraj.end())
                        .lineToLinearHeading(travelPoint.getPose())
                                .build();
        Trajectory leftBackdropTraj = drive.trajectoryBuilder(travelTraj.end())
                .strafeTo(backdropPoint.getPos())
                .build();
        Trajectory audienceTraj = drive.trajectoryBuilder(mediaryTraj.end())
                .lineToLinearHeading(mediaryPoint.getPose())
                .build();
        Trajectory tempParkTrajLeft = drive.trajectoryBuilder(leftBackdropTraj.end())
                .strafeTo(tempParkPoint.getPos())
                .build();
        Trajectory tempParkTraj = drive.trajectoryBuilder(backdropTraj.end())
                .strafeTo(tempParkPoint.getPos())
                .build();

        /*
        after BackdropTraj:
        -select AprilTag to focus on and move to that position
        -move arm to deposit pixels
        -lower arm

        Loop the following:
        reverse back to x-coord and heading of travelTraj target
        reverse to pixel stack
        intake one pixel, then intake another pixel,
        repeat the already programmed auto steps of travelTraj and backdropTraj
        deposit pixel

        Long term idea:
        create option to wait further back from the backdrop and wait until all 3 april tags are detected to avoid robot collision
         */

        //Goal for comp 1: deposit both pixels on board + cycle once; feasible??





        if(LEFT){
            DashboardUtil.previewTrajectories(FtcDashboard.getInstance(), spikeTraj, mediaryTraj, centerTraj, travelTraj, leftBackdropTraj);
        } else {
            DashboardUtil.previewTrajectories(FtcDashboard.getInstance(), spikeTraj, mediaryTraj, backdropTraj);
        }

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(spikeTraj);
        drive.followTrajectory(mediaryTraj);
        if(!LEFT) {
            drive.followTrajectory(backdropTraj);
            armServo.setPosition(0.275);
            operationTimer.reset();
            sleep(750);
            armServo.setPosition(0.04);
            drive.followTrajectory(tempParkTraj);
        }
        if(LEFT){
            drive.followTrajectory(audienceTraj);
            drive.followTrajectory(centerTraj);
            drive.followTrajectory(travelTraj);
            drive.followTrajectory(leftBackdropTraj);
            armServo.setPosition(0.275);
            sleep(750);
            armServo.setPosition(0.04);
            drive.followTrajectory(tempParkTrajLeft);
        }
    }
}
