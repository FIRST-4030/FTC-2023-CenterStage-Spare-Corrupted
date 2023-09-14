package org.firstinspires.ftc.teamcode.robot.camera;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.ArrayList;
import java.util.List;


public class ComputerVision {

    AprilTagProcessor aprilTagProcessor;
    AprilTagProcessor.Builder aprilTagBuilder;
    TfodProcessor tensorFlowProcessor;
    TfodProcessor.Builder tensorFlowBuilder;
    VisionPortal visionPortal;
    VisionPortal.Builder visionPortalBuilder;
    HardwareMap hardwareMap;

    public List<Recognition> tensorFlowRecognitions;
    public ArrayList<AprilTagDetection> aprilTagDetections;


    public ComputerVision() {
        aprilTagBuilder = new AprilTagProcessor.Builder();
        tensorFlowBuilder = new TfodProcessor.Builder();

    }
    private void initAprilTag() { //Build the AprilTag vision processor
        aprilTagProcessor = aprilTagBuilder.build();
    }

    private void initTensorFlow() { //Build the TensorFlow vision processor
        tensorFlowProcessor = tensorFlowBuilder.build();
    }

    public void initVisionPortal() { //runs init on vision processors and builds the VisionPortal
        initTensorFlow();
        initAprilTag();
        visionPortal = new VisionPortal.Builder()
                .addProcessors(aprilTagProcessor, tensorFlowProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setAutoStopLiveView(true)
                .build();

    }

    public void update() {
        tensorFlowRecognitions = tensorFlowProcessor.getRecognitions();
        aprilTagDetections = aprilTagProcessor.getDetections();

    }






}
