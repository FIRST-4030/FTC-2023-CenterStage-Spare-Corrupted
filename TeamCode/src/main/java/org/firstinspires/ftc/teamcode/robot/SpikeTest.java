package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.drives.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.gamepad.gamepad.InputAutoMapper;
import org.firstinspires.ftc.teamcode.gamepad.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.general.Pose2dWrapper;
import org.firstinspires.ftc.teamcode.drive.drives.CustomMecanumDrive;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.Endpoint;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.util.ArrayList;
import java.util.HashMap;

@Config
@Autonomous(name = "SpikeTest")
public class SpikeTest extends LinearOpMode {
    public int spike = 2;
    public static double SPIKE_POINT_X = 12;
    public static double SPIKE_POINT_Y = -34.5;
    public static double HEADING = 90;
    double BACKDROPY = -35;
    public static double POINTX = 45;
    public static double POINTY = -60;
    public static double POINTHEADING = 90;
    public static boolean audience = false;
    double BACKDROPX = 50.75;

    public ArrayList<Endpoint> endpoints = new ArrayList<Endpoint>();

    public Pose2dWrapper startPose = new Pose2dWrapper(15, -62.5, Math.toRadians(90));
    public Pose2dWrapper resetPose = new Pose2dWrapper(13, -51.5, -90);
    public Pose2dWrapper mediaryPose = new Pose2dWrapper(15, -50.5, 0);
    public Pose2dWrapper audiencePose = new Pose2dWrapper(-58, -41.5, 0);
    public Pose2dWrapper backdropPose = new Pose2dWrapper(36, -37.5, 0);
    public Pose2dWrapper centerPose = new Pose2dWrapper(-58, -8.5, 0);
    public Pose2dWrapper avoidancePose = new Pose2dWrapper(30.5 , -13.5, 90);
    public Pose2dWrapper tempParkPose = new Pose2dWrapper(48, -61.5, 0);
    //X values get wonky here, as invertLeft is ran on all Endpoints used on the left starting point,
    //so numbers are less than would be expected and sometimes greater than 70, however invertLeft() clears this up,
    //decided to do this for readability in the if(LEFT) statement
    public Pose2dWrapper travelPose = new Pose2dWrapper(35, -8.5, 0);
    public Pose2dWrapper aprilTagPose = new Pose2dWrapper(50, -37, 0);
    public Pose2dWrapper pixelPose = new Pose2dWrapper(-58, -36.5, 0);
    public Pose2dWrapper postPixelPose = new Pose2dWrapper(-60, -36.5, 0);


    ComputerVision vision;
    ArrayList<AprilTagDetection> aprilTagDetections;
    HashMap<Integer, AprilTagPoseFtc> aprilTagTranslations = new HashMap<>();
    InputHandler inputHandler;
    Servo armServo;

    ElapsedTime operationTimer = new ElapsedTime();
    boolean inputComplete = false;
    boolean isBlue = false;
    Pose2d robotPose;

    Servo leftFlipper;
    Servo rightFlipper;
    DcMotorSimple intake;
    ElapsedTime runtime = new ElapsedTime();
    int i = 1;




    @Override
    public void runOpMode() throws InterruptedException {
        runtime.reset();
        inputHandler = InputAutoMapper.normal.autoMap(this);
        while(inputComplete == false){
            inputHandler.loop();
            if(inputHandler.up("D1:DPAD_LEFT")){
                isBlue = !isBlue;
            }
            if(inputHandler.up("D1:DPAD_RIGHT")){
                audience = !audience;
            }
            if(inputHandler.up("D1:X")){
                inputComplete = true;
            }
            telemetry.addData("is Blue:", isBlue);
            telemetry.addData("is Near Audience:", audience);
            telemetry.addData("Press X to finalize values", inputComplete);
            telemetry.update();
        }
        NewMecanumDrive drive = new NewMecanumDrive(hardwareMap);
        vision = new ComputerVision(hardwareMap);
        armServo = hardwareMap.get(Servo.class, "Arm");

        leftFlipper = hardwareMap.get(Servo.class, "leftHook");
        rightFlipper = hardwareMap.get(Servo.class, "rightHook");

        leftFlipper.setPosition(0.999);
        rightFlipper.setPosition(0.01);

        intake = hardwareMap.get(DcMotorSimple.class, "Intake");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setPower(0);

        while(opModeInInit()) {
            vision.updateTensorFlow();
            spike = vision.checkSpike(isBlue, audience);
            sleep(1);
            telemetry.addData("spike: ", spike);
            telemetry.update();
        }


        if (isStopRequested()) return;
        switch (spike) {
            case 1:
                SPIKE_POINT_X = 3.7;
                SPIKE_POINT_Y = -34.5;
                HEADING = 115;
                aprilTagPose.y = -28.3;
                break;
            case 2:
                SPIKE_POINT_X = 11;
                SPIKE_POINT_Y = -36.5;
                HEADING = 90;
                aprilTagPose.y = -35.5;
                break;
            case 3:
                SPIKE_POINT_X = 17.5;
                SPIKE_POINT_Y = -34.5;
                HEADING = 65;
                aprilTagPose.y = -42.4;
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
        Endpoint aprilPoint = new Endpoint(aprilTagPose.x, aprilTagPose.y, aprilTagPose.heading);
        Endpoint pixelPoint = new Endpoint(pixelPose.x, pixelPose.y, pixelPose.heading);
        Endpoint postPixelPoint = new Endpoint(postPixelPose.x, postPixelPose.y, postPixelPose.heading);



        if(audience){
            startPose.x += 24;
            startPose.x *= -1;
            tempParkPoint.pose.y = -13.5;
            BACKDROPX = 47.75;




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
        if(isBlue){
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
            aprilPoint.invertSides();
            pixelPoint.invertSides();
            postPixelPoint.invertSides();
        }



        drive.setPoseEstimate(startPose.toPose2d());

        Trajectory spikeTraj = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineTo(spikePoint.getPos(), Math.toRadians(spikePoint.getHeading()))
                .build();
        Trajectory mediaryTraj = drive.trajectoryBuilder(spikeTraj.end())
                .strafeTo(mediaryPoint.getPos())
                .build();
        Trajectory backdropTraj = drive.trajectoryBuilder(mediaryTraj.end())
                .splineTo(backdropPoint.getPos(), Math.toRadians(backdropPoint.getHeading()))
                .build();


        //modify to strafe w/ heading of 0
        //reverse to pixels
        //intake pixel (turn intake on, use flippers)
        //disable intake

        Trajectory audienceTraj = drive.trajectoryBuilder(mediaryTraj.end())
                .lineToLinearHeading(audiencePoint.getPose())
                .build();
        Trajectory pixelTraj = drive.trajectoryBuilder(audienceTraj.end())
                .strafeTo(pixelPoint.getPos())
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

        outputLog(drive); //1
        drive.followTrajectory(spikeTraj);
        outputLog(drive); //2
        drive.followTrajectory(mediaryTraj);
        outputLog(drive); //3
        vision.tensorFlowProcessor.shutdown();
        if(!audience) {
            drive.followTrajectory(backdropTraj);
            outputLog(drive);
            while(aprilTagTranslations.get(5) == null){
                vision.updateAprilTags();
                aprilTagTranslations = vision.getTranslationToTags();
                robotPose = vision.localize(5, true);
            }
            drive.setPoseEstimate(robotPose);
            Trajectory aprilTagTraj = drive.trajectoryBuilder(robotPose)
                    .strafeTo(aprilPoint.getPos(),
                    NewMecanumDrive.getVelocityConstraint(20, 1.85, 13.5),
                    NewMecanumDrive.getAccelerationConstraint(20))
                    .build();
            Trajectory tempParkTraj = drive.trajectoryBuilder(aprilTagTraj.end())
                .strafeTo(tempParkPoint.getPos())
                    .build();
            drive.followTrajectory(aprilTagTraj);
            outputLog(drive);
            armServo.setPosition(0.275);
            sleep(2750);
            armServo.setPosition(0.04);
            outputLog(drive);
            drive.followTrajectory(tempParkTraj);
            outputLog(drive);
        }

        if(audience){
            drive.followTrajectory(audienceTraj);
            outputLog(drive); //4
            drive.followTrajectory(pixelTraj);
            outputLog(drive); //5
            vision.setActiveCameraTwo();
            while(aprilTagTranslations.get(8) == null){
                vision.updateAprilTags();
                aprilTagTranslations = vision.getTranslationToTags();
                robotPose = vision.localize(8, false);
            }
            drive.setPoseEstimate(robotPose);
            outputLog(drive); //6
            Trajectory postPixelTraj =  drive.trajectoryBuilder(robotPose)
                    .lineToLinearHeading(postPixelPoint.getPose(),
                            NewMecanumDrive.getVelocityConstraint(10, 1.85, 13.5),
                            NewMecanumDrive.getAccelerationConstraint(10))
                                    .build();
            drive.followTrajectory(postPixelTraj);
            outputLog(drive);//7

            intake.setPower(1);
            leftFlipper.setPosition(0.4);
            rightFlipper.setPosition(0.6);
            sleep(650);
            leftFlipper.setPosition(0.999);
            rightFlipper.setPosition(0.01);

            Trajectory centerTraj = drive.trajectoryBuilder(postPixelTraj.end())
                    .strafeTo(centerPoint.getPos())
                    .build();
            Trajectory travelTraj = drive.trajectoryBuilder(centerTraj.end())
                    .lineToLinearHeading(travelPoint.getPose())
                    .build();
            Trajectory leftBackdropTraj = drive.trajectoryBuilder(travelTraj.end())
                    .strafeTo(backdropPoint.getPos())
                    .build();
            drive.followTrajectory(centerTraj);
            outputLog(drive);//8
            drive.followTrajectory(travelTraj);
            outputLog(drive);//9
            drive.followTrajectory(leftBackdropTraj);
            outputLog(drive);
            intake.setPower(0);
            vision.setActiveCameraOne();
            while(aprilTagTranslations.get(5) == null){
                vision.updateAprilTags();
                aprilTagTranslations = vision.getTranslationToTags();
                robotPose = vision.localize(5, true);
            }
            drive.setPoseEstimate(robotPose);
            outputLog(drive);
            Trajectory aprilTagTraj = drive.trajectoryBuilder(robotPose)
                    .strafeTo(aprilPoint.getPos(),
                            NewMecanumDrive.getVelocityConstraint(20, 1.85, 13.5),
                            NewMecanumDrive.getAccelerationConstraint(20))
                    .build();
            Trajectory tempParkTrajLeft = drive.trajectoryBuilder(aprilTagTraj.end())
                    .strafeTo(tempParkPoint.getPos())
                    .build();
            drive.followTrajectory(aprilTagTraj);
            outputLog(drive);
            armServo.setPosition(0.275);
            sleep(2750);
            armServo.setPosition(0.04);
            drive.followTrajectory(tempParkTrajLeft);
            outputLog(drive);
        }
    }
    public void outputLog(NewMecanumDrive drive){
        RobotLog.d("WAY: Current Robot Pose Estimate and time: X: %.03f Y: %.03f Heading: %.03f ms: %.03f iteration: %d", drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), Math.toDegrees(drive.getPoseEstimate().getHeading()), runtime.milliseconds(), i);
        i++;
    }
}
