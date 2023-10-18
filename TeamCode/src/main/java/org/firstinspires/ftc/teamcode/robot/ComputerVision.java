package org.firstinspires.ftc.teamcode.robot;

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

    public List<Recognition> tensorFlowRecognitions;
    public ArrayList<AprilTagDetection> aprilTagDetections;

    public ComputerVision(HardwareMap hardwareMap) {

        aprilTagBuilder = new AprilTagProcessor.Builder();
        tensorFlowBuilder = new TfodProcessor.Builder();

    }

    public void initVisionPortal(HardwareMap hardwareMap) { //runs init on vision processors and builds the VisionPortal
        tensorFlowProcessor = tensorFlowBuilder.build();
        aprilTagProcessor = aprilTagBuilder.build();
        visionPortal = new VisionPortal.Builder()
                .addProcessors(aprilTagProcessor, tensorFlowProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setAutoStopLiveView(true)
                .enableLiveView(true)
                .build();

    }

    public void update() {
        tensorFlowRecognitions = tensorFlowProcessor.getRecognitions();
        aprilTagDetections = aprilTagProcessor.getDetections();
    }
}
