package org.firstinspires.ftc.teamcode.robot;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;


public class ComputerVision{

    public AprilTagProcessor aprilTagProcessor;
    //AprilTagProcessor.Builder aprilTagBuilder;
    public TfodProcessor tensorFlowProcessor;
    //TfodProcessor.Builder tensorFlowBuilder;
    VisionPortal visionPortal;
    //VisionPortal.Builder visionPortalBuilder;
    String[] labels = {"Blue Prop", "Red Prop"};

    public List<Recognition> tensorFlowRecognitions;
    public ArrayList<AprilTagDetection> aprilTagDetections;
    public int spike = 1;
    Pose2d robotPose = new Pose2d();
    WebcamName webcam1, webcam2;


    public static final ArrayList<Pose2d> aprilTagPoses = new ArrayList<>(Arrays.asList(
            new Pose2d(62, 41.4, 0), //0
            new Pose2d(62, 35.5, 0), //1
            new Pose2d(62, 29.3, 0), //2
            new Pose2d(62, -29.3, 0), //3
            new Pose2d(62, -35.5, 0), //4
            new Pose2d(62, -41.4, 0), //5
            new Pose2d(0, 0, 0), //6
            new Pose2d(-70.06, -41, 0), //7
            new Pose2d(0, 0, 0), //8
            new Pose2d(0, 0, 0)  //9
    ));

    AprilTagPoseFtc currentTagTranslation;
    HashMap<Integer, AprilTagPoseFtc> aprilTagTranslations = new HashMap<>();

    public ComputerVision(HardwareMap hardwareMap){

        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");
        CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(webcam1, webcam2);
        tensorFlowProcessor = new TfodProcessor.Builder()
                .setModelFileName("/sdcard/FIRST/tflitemodels/model_20231101_085815.tflite")
                .setModelLabels(labels)
                .build();
        aprilTagProcessor = new AprilTagProcessor.Builder()
                //.setLensIntrinsics(952.837, 952.837, 622.758, 398.223)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();
        tensorFlowProcessor.setMinResultConfidence(0.60f);
        visionPortal = new VisionPortal.Builder()
                .addProcessors(aprilTagProcessor, tensorFlowProcessor)
                .setCamera(switchableCamera)
                .setCameraResolution(new Size(1280, 720))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(true)
                .build();
    }



    public void update() {
        tensorFlowRecognitions = tensorFlowProcessor.getRecognitions();
        aprilTagDetections = aprilTagProcessor.getDetections();
    }

    public int checkSpike(boolean isBlue, boolean audience) {
        /*
        Logic for determining spike,
        first checks to see which side the robot is on, then it iterates through all the recognitions
        until it sees a blue prop, assuming there is only one blue prop.
        It then looks to see the coordinates of the left side (decided arbitrarily, should change)
        It then checks to see which third of the camera the coordinate is in and bases the target spike off of that.
         */
        if (isBlue) {
            tensorFlowRecognitions.forEach(
                    (Recognition) -> {
                        if (Recognition.getLabel() == "Blue Prop") {
                            if(audience) {
                                if (Recognition.getLeft() <= 443) {
                                    spike = 3;
                                } else if (Recognition.getLeft() <= 885) {
                                    spike = 2;
                                } else if (Recognition.getLeft() <= 1280){
                                    spike = 1;
                                }
                            } else {
                                if (Recognition.getLeft() <= 198) {
                                    spike = 3;
                                } else if (Recognition.getLeft() <= 640) {
                                    spike = 2;
                                } else if (Recognition.getLeft() <= 1280)  {
                                    spike = 1;
                                }
                            }
                        }
                    });
        }
        else {
            tensorFlowRecognitions.forEach(
                    (Recognition) -> {
                        if (Recognition.getLabel() == "Red Prop") {
                            if(audience) {
                                if (Recognition.getLeft() <= 443) {
                                    spike = 1;
                                } else if (Recognition.getLeft() <= 885) {
                                    spike = 2;
                                } else if (Recognition.getLeft() <= 1280){
                                    spike = 3;
                                }
                            } else{
                                if (Recognition.getLeft() <= 198) {
                                    spike = 1;
                                } else if (Recognition.getLeft() <= 640) {
                                    spike = 2;
                                } else if (Recognition.getLeft() <= 1280){
                                    spike = 3;
                                }
                            }
                        }
                    });
        }
        return spike;
    }

        public HashMap<Integer, AprilTagPoseFtc> getTranslationToTags(){
            aprilTagTranslations.clear();
            aprilTagDetections.forEach((AprilTagDetection) -> {
                aprilTagTranslations.put(AprilTagDetection.id, AprilTagDetection.ftcPose);
            });
            return aprilTagTranslations;
        }

        public Pose2d localize(int id, boolean frontCam){
            try {
                currentTagTranslation = getTranslationToTags().get(id);
                Pose2d aprilTagPose = aprilTagPoses.get(id-1);
                robotPose = new Pose2d(frontCam ? aprilTagPose.getX() - currentTagTranslation.y - 8 : aprilTagPose.getX() + currentTagTranslation.y + 8,
                        aprilTagPose.getY() + currentTagTranslation.x,
                        Math.toRadians(0));
                return robotPose;
            } catch(Exception e) {
                    robotPose = new Pose2d(10,10,10);
                    return robotPose;
            }
        }

        public List<Recognition> getTensorFlowRecognitions () {
            return tensorFlowRecognitions;
        }

        public ArrayList<AprilTagDetection> getAprilTagDetections () {
            return aprilTagDetections;
        }
        public void setActiveCameraOne() {
            visionPortal.setActiveCamera(webcam1);
        }
    public void setActiveCameraTwo() {
        visionPortal.setActiveCamera(webcam2);
    }
    }