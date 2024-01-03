package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.vision.VisionPortal.CameraState.OPENING_CAMERA_DEVICE;

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
    double spikePointX = 12;
    double spikePointY = -34.5;
    public  double spikeHeading = 90;
    public boolean audience = false;
    double backdropX = 50.75;
    int backdropCenterAT = 5;
    int audienceAT = 8;


    public Pose2dWrapper startPose = new Pose2dWrapper(15, -62.5, Math.toRadians(90));
    public Pose2dWrapper mediaryPose = new Pose2dWrapper(15, -50.5, 0);
    public Pose2dWrapper backdropPose = new Pose2dWrapper(33, -36.5, 0);
    public Pose2dWrapper centerPose = new Pose2dWrapper(-57, -7.5, 0);
    public Pose2dWrapper outerCenterPose = new Pose2dWrapper(-40, -58, 0);
    public Pose2dWrapper tempParkPose = new Pose2dWrapper(48, -61.5, 0);
    public Pose2dWrapper travelPose = new Pose2dWrapper(25, -10, 0);
    public Pose2dWrapper outerTravelPose = new Pose2dWrapper(24, -58, 0);
    public Pose2dWrapper aprilTagPose = new Pose2dWrapper(52, -37, 0);
    public Pose2dWrapper pixelPose = new Pose2dWrapper(-53, -37, 0);
    public Pose2dWrapper postPixelPose = new Pose2dWrapper(-60, -36.5, 0);
    public Pose2dWrapper avoidancePose = new Pose2dWrapper(-58 , -36.5, 0);
    public Pose2dWrapper secondCollectionPose = new Pose2dWrapper(-57.75, -10, 0);
    public Pose2dWrapper finalDepositPose = new Pose2dWrapper(52, -39, 0);
    public Pose2dWrapper secondTravelPose = new Pose2dWrapper(25, -10, 0);
    public Pose2dWrapper preSecondCollectionPose = new Pose2dWrapper(-42.25, -10, 0);


    ComputerVision vision;
    AprilTagPoseFtc[] aprilTagTranslations = new AprilTagPoseFtc[11];
    InputHandler inputHandler;
    Servo armServo;

    boolean inputComplete = false;
    boolean isBlue = false;
    Pose2d robotPose;

    Servo leftFlipper;
    Servo rightFlipper;
    DcMotorSimple intake;
    ElapsedTime runtime = new ElapsedTime();
    int i = 1; //used as an iterator for outputLog()
    boolean returned = false;
    int pixelSide = 2;
    String pixelSideInEnglish;
    long sleepTime = 10;



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
            if(inputHandler.up("D1:DPAD_UP")){
                if(pixelSide == 3){
                    pixelSide = 1;
                } else {
                    pixelSide++;
                }
            }
            switch(pixelSide){
                case 1:
                    pixelSideInEnglish = "Left";
                    break;
                case 2:
                    pixelSideInEnglish = "Center";
                case 3:
                    pixelSideInEnglish = "Right";
            }
            if(inputHandler.up("D1:X")){
                inputComplete = true;
            }
            telemetry.addData("is Blue:", isBlue);
            telemetry.addData("is Near Audience:", audience);
            telemetry.addData("Yellow Pixel Deposit Offset: ", pixelSideInEnglish);
            telemetry.addData("Press X to finalize values", inputComplete);
            telemetry.update();
        }
        vision = new ComputerVision(hardwareMap);
        while(vision.visionPortal.getCameraState() == OPENING_CAMERA_DEVICE){

        }
        vision.setActiveCameraOne();
        armServo = hardwareMap.get(Servo.class, "Arm");
        NewMecanumDrive drive = new NewMecanumDrive(hardwareMap);

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
            sleep(sleepTime);
            telemetry.addData("spike: ", spike);
            telemetry.update();
        }


        if (isStopRequested()) return;
        if(isBlue) {
            switch (spike) {
                case 3:
                    spikePointX = 3.7;
                    spikePointY = -34.5;
                    spikeHeading = 115;
                    aprilTagPose.y = -28.3;
                    break;
                case 2:
                    spikePointX = 9;
                    spikePointY = -35.5;
                    spikeHeading = 105;
                    aprilTagPose.y = -35.5;
                    break;
                case 1:
                    spikePointX = 17.5;
                    spikePointY = -34.5;
                    spikeHeading = 65;
                    aprilTagPose.y = -42.4;
                    break;
            }
        } else {
            switch (spike) {
                case 1:
                    spikePointX = 3.7;
                    spikePointY = -34.5;
                    spikeHeading = 115;
                    aprilTagPose.y = -28.3;
                    break;
                case 2:
                    spikePointX = 9;
                    spikePointY = -35.5;
                    spikeHeading = 105;
                    aprilTagPose.y = -35.5;
                    break;
                case 3:
                    spikePointX = 17.5;
                    spikePointY = -34.5;
                    spikeHeading = 65;
                    aprilTagPose.y = -42.4;
                    break;
            }
        }
        /*if(pixelSide == 1){
                aprilTagPose.y += 0.5;
        }
        if(pixelSide == 3){
            aprilTagPose.y -= 0.5;
        }*/

        Pose2dWrapper spikePose = new Pose2dWrapper(spikePointX, spikePointY, spikeHeading);


        if(audience){
            startPose.x = -39;
            tempParkPose.y = -10;
            backdropX = 47.75;

            spikePose.x -= 46;
            if(spike == 2){
                spikePose.x += 4;
                spikePose.heading = 75;
            }
            mediaryPose.x = -(mediaryPose.x + 34);
        }
        if(isBlue){
            startPose.y *= -1;
            startPose.heading *= -1;
            audienceAT = 9;
            backdropCenterAT = 2;

            spikePose.y *= -1;
            spikePose.heading *= -1;
            mediaryPose.y *= -1;
            mediaryPose.heading *= -1;
            backdropPose.y *= -1;
            backdropPose.heading *= -1;
            centerPose.y *= -1;
            centerPose.heading *= -1;
            avoidancePose.y *= -1;
            avoidancePose.heading *= -1;
            travelPose.y *= -1;
            travelPose.heading *= -1;
            tempParkPose.y *= -1;
            tempParkPose.heading *= -1;
            aprilTagPose.y *= -1;
            aprilTagPose.heading *= -1;
            pixelPose.y *= -1;
            pixelPose.heading *= -1;
            postPixelPose.y *= -1;
            postPixelPose.heading *= -1;
            outerTravelPose.y *= -1;
            outerTravelPose.heading *= -1;
            outerCenterPose.y *= -1;
            outerCenterPose.heading *= -1;
            secondCollectionPose.y *= -1;
            secondCollectionPose.heading *= -1;
            finalDepositPose.y *= -1;
            finalDepositPose.heading *= -1;
            preSecondCollectionPose.y *= -1;
            preSecondCollectionPose.heading *= -1;
            secondTravelPose.y *= -1;
            secondTravelPose.heading *= -1;
        }

        drive.setPoseEstimate(startPose.toPose2d());

        Trajectory spikeTraj = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineTo(spikePose.toPose2d().vec(), Math.toRadians(spikePose.heading))
                .build();
        Trajectory mediaryTraj = drive.trajectoryBuilder(spikeTraj.end(), true)
                .splineTo(mediaryPose.toPose2d().vec(), Math.toRadians(180-mediaryPose.heading))
                    .build();
        Trajectory backdropTraj = drive.trajectoryBuilder(mediaryTraj.end())
                .splineTo(backdropPose.toPose2d().vec(), Math.toRadians(backdropPose.heading))
                .build();

        Trajectory pixelTraj = drive.trajectoryBuilder(mediaryTraj.end())
                .strafeTo(pixelPose.toPose2d().vec())
                .build();

        outputLog(drive); //1
        drive.followTrajectory(spikeTraj);
        if(!audience) {
            outputLog(drive); //2
            drive.followTrajectory(mediaryTraj);
            outputLog(drive); //3
            drive.followTrajectory(backdropTraj);
            Trajectory outerTraj = depositPixel(drive, false);
            drive.followTrajectory(outerTraj);
            outputLog(drive);
            vision.tensorFlowProcessor.shutdown();
            Trajectory returnTraj = collectPixelAudience(drive, 2);
            drive.followTrajectory(returnTraj);
            outputLog(drive);
            Trajectory parkTraj = depositPixel(drive, true);
            drive.followTrajectory(parkTraj);


        }


        if(audience){
            outputLog(drive); //2
            //drive.followTrajectory(postSpikeTraj);
            vision.setActiveCameraTwo();
            drive.followTrajectory(mediaryTraj);
            outputLog(drive); //3
            drive.followTrajectory(pixelTraj);
            outputLog(drive); //4
            vision.tensorFlowProcessor.shutdown();
            Trajectory avoidanceTraj = collectPixelAudience(drive, 1);
            Trajectory centerTraj = drive.trajectoryBuilder(avoidanceTraj.end())
                    .strafeTo(centerPose.toPose2d().vec())
                            .build();
            Trajectory travelTraj = drive.trajectoryBuilder(centerTraj.end())
                    .splineTo(travelPose.toPose2d().vec(), Math.toRadians(travelPose.heading))
                    .splineToConstantHeading(backdropPose.toPose2d().vec(), Math.toRadians(backdropPose.heading))
                            .build();
            drive.followTrajectory(avoidanceTraj);
            outputLog(drive);
            drive.followTrajectory(centerTraj);
            outputLog(drive);
            drive.followTrajectory(travelTraj);
            outputLog(drive);
            Trajectory secondCollectionTraj = depositPixel(drive, true);
            Trajectory precisionCollectionTraj = drive.trajectoryBuilder(secondCollectionTraj.end())
                            .strafeTo(secondCollectionPose.toPose2d().vec())
                                    .build();
            telemetry.addData("Heading: ", drive.getExternalHeading());
            outputLog(drive);
            drive.followTrajectory(secondCollectionTraj);
            drive.followTrajectory(precisionCollectionTraj);
            outputLog(drive);
            for(int i = 0; i < 2; i++ ) {
                intake.setPower(1);
                leftFlipper.setPosition(0.4);
                rightFlipper.setPosition(0.6);
                sleep(650);
                leftFlipper.setPosition(0.999);
                rightFlipper.setPosition(0.01);
                if(i+1 < 2){
                    sleep(650);
                }
            }
            Trajectory secondReturnTraj = drive.trajectoryBuilder(precisionCollectionTraj.end())
                    .splineTo(secondTravelPose.toPose2d().vec(), Math.toRadians(travelPose.heading))
                    .addDisplacementMarker(() -> {
                        intake.setPower(0);
                        armServo.setPosition(0.285);
                        })
                    .splineToConstantHeading(finalDepositPose.toPose2d().vec(), Math.toRadians(finalDepositPose.heading))
                    .build();
            drive.followTrajectory(secondReturnTraj);
            outputLog(drive);
            armServo.setPosition(0.04);
            Trajectory parkTraj = drive.trajectoryBuilder(secondReturnTraj.end())
                    .strafeTo(tempParkPose.toPose2d().vec())
                    .build();
            sleep(250);
            drive.followTrajectory(parkTraj);
            outputLog(drive);
        }
    }
    public Trajectory depositPixel(NewMecanumDrive drive, boolean fin){
        Trajectory tempTrajDeposit;
        intake.setPower(0);
        vision.setActiveCameraOne();
        armServo.setPosition(0.285);
        while(aprilTagTranslations[backdropCenterAT] == null){
            vision.updateAprilTags();
            aprilTagTranslations = vision.getTranslationToTags();
            robotPose = vision.localize(backdropCenterAT, true);
            sleep(sleepTime);
        }
        drive.setPoseEstimate(robotPose);
        outputLog(drive); //9
        Trajectory aprilTagTraj = drive.trajectoryBuilder(robotPose)
                .strafeTo(aprilTagPose.toPose2d().vec(),
                        NewMecanumDrive.getVelocityConstraint(30, 2, 13.5),
                        NewMecanumDrive.getAccelerationConstraint(30))
                .build();
        drive.followTrajectory(aprilTagTraj);
        outputLog(drive); //10
        armServo.setPosition(0.04);
        if(!fin) {
            tempTrajDeposit = drive.trajectoryBuilder(aprilTagTraj.end(), true)
                    .splineTo(outerTravelPose.toPose2d().vec(), Math.toRadians(180-outerTravelPose.heading))
                    .splineTo(outerCenterPose.toPose2d().vec(), Math.toRadians(180-outerCenterPose.heading))
                    .splineTo(pixelPose.toPose2d().vec(), Math.toRadians(180-pixelPose.heading))
                    .build();
        }
        else {
            if(audience && returned == false){
                tempTrajDeposit = drive.trajectoryBuilder(aprilTagTraj.end(), true)
                        .splineToConstantHeading(secondTravelPose.toPose2d().vec(), Math.toRadians(180-tempParkPose.heading))
                        .addDisplacementMarker(() -> {
                            telemetry.addData("Heading: ", drive.getExternalHeading());
                            telemetry.update();
                        })
                        .splineToConstantHeading(preSecondCollectionPose.toPose2d().vec(), Math.toRadians(180-preSecondCollectionPose.heading))
                        .build();
                returned = true;
            } else {
                tempTrajDeposit = drive.trajectoryBuilder(aprilTagTraj.end())
                        .strafeTo(tempParkPose.toPose2d().vec())
                        .build();
            }
        }
        return tempTrajDeposit;
    }

    public Trajectory collectPixelAudience(NewMecanumDrive drive, int numPixels){
        Trajectory centerTraj;
        outputLog(drive); //5
        while(aprilTagTranslations[audienceAT] == null){
            vision.updateAprilTags();
            aprilTagTranslations = vision.getTranslationToTags();
            robotPose = vision.localize(audienceAT, false);
            sleep(sleepTime);
        }
        drive.setPoseEstimate(robotPose);
        outputLog(drive); //6
        Trajectory postPixelTraj =  drive.trajectoryBuilder(robotPose)
                .lineToConstantHeading(postPixelPose.toPose2d().vec(),
                        NewMecanumDrive.getVelocityConstraint(30, 1.85, 13.5),
                        NewMecanumDrive.getAccelerationConstraint(30))
                .build();
        if (audience) {
            centerTraj = drive.trajectoryBuilder(postPixelTraj.end())
                    .strafeTo(avoidancePose.toPose2d().vec())
                    .build();
        } else{
            centerTraj = drive.trajectoryBuilder(postPixelTraj.end())
                    .splineTo(outerCenterPose.toPose2d().vec(), Math.toRadians(outerCenterPose.heading))
                    .splineTo(outerTravelPose.toPose2d().vec(), Math.toRadians(outerTravelPose.heading))
                    .splineToConstantHeading(backdropPose.toPose2d().vec(), Math.toRadians(backdropPose.heading))
                    .build();
        }


        drive.followTrajectory(postPixelTraj);
        outputLog(drive);//7
        for(int i = 0; i < numPixels; i++ ) {
            intake.setPower(1);
            leftFlipper.setPosition(0.4);
            rightFlipper.setPosition(0.6);
            sleep(650);
            leftFlipper.setPosition(0.999);
            rightFlipper.setPosition(0.01);
            if(i+1 < numPixels){
                sleep(650);
            }
        }
        return centerTraj;
    }
    public void outputLog(NewMecanumDrive drive){
        RobotLog.d("WAY: Current Robot Pose Estimate and time: X: %.03f Y: %.03f Heading: %.03f ms: %.03f iteration: %d", drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), Math.toDegrees(drive.getPoseEstimate().getHeading()), runtime.milliseconds(), i);
        i++;
    }
}
