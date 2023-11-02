package org.firstinspires.ftc.teamcode.robot;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.tensorflow.lite.support.common.TensorProcessor;

import java.util.ArrayList;
import java.util.List;


public class ComputerVision{

    AprilTagProcessor aprilTagProcessor;
    AprilTagProcessor.Builder aprilTagBuilder;
    TfodProcessor tensorFlowProcessor;
    TfodProcessor.Builder tensorFlowBuilder;
    VisionPortal visionPortal;
    VisionPortal.Builder visionPortalBuilder;

    public List<Recognition> tensorFlowRecognitions;
    public ArrayList<AprilTagDetection> aprilTagDetections;
    public int spike = 1;
    public String[] labels = {"Blue Prop", "Red Prop"};

    public ComputerVision(HardwareMap hardwareMap){

        aprilTagBuilder = new AprilTagProcessor.Builder().setLensIntrinsics(952.837, 952.837, 622.758, 398.223);
        tensorFlowBuilder = new TfodProcessor.Builder();
        tensorFlowProcessor = new TfodProcessor.Builder()
                .setModelFileName("/sdcard/FIRST/tflitemodels/model_20231101_085815.tflite")
                .setModelLabels(labels)
                .build();
        aprilTagProcessor = aprilTagBuilder.build();
        tensorFlowProcessor.setMinResultConfidence(0.60f);
        visionPortal = new VisionPortal.Builder()
                .addProcessors(aprilTagProcessor, tensorFlowProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(1280, 720))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(true)
                .build();

    }


    public void update() {
        tensorFlowRecognitions = tensorFlowProcessor.getRecognitions();
        aprilTagDetections = aprilTagProcessor.getDetections();
    }

    public int checkSpike(boolean isBlue) {
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
                            if (Recognition.getLeft() <= 369) {
                                spike = 3;
                            } else if (370 < Recognition.getLeft() && Recognition.getLeft() <= 759) {
                                spike = 2;
                            } else if (760 < Recognition.getLeft() && Recognition.getLeft() <= 1280) {
                                spike = 1;
                            }
                        }
                    });
        }
        else {
            tensorFlowRecognitions.forEach(
                    (Recognition) -> {
                        if (Recognition.getLabel() == "Red Prop") {
                            if (Recognition.getLeft() <= 369) {
                                spike = 1;
                            } else if (370 < Recognition.getLeft() && Recognition.getLeft() <= 759) {
                                spike = 2;
                            } else if (760 < Recognition.getLeft() && Recognition.getLeft() <= 1280) {
                                spike = 3;
                            }
                        }
                    });
        }
        return spike;
    }

        public List<Recognition> getTensorFlowRecognitions () {
            return tensorFlowRecognitions;
        }

        public ArrayList<AprilTagDetection> getAprilTagDetections () {
            return aprilTagDetections;
        }
    }