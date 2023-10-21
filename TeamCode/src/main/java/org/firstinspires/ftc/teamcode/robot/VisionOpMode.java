package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.ComputerVision;

@TeleOp
public class VisionOpMode extends OpMode {
    public ComputerVision computerVision;
    boolean isBlue = false;
    @Override
    public void init() {
        computerVision = new ComputerVision(hardwareMap);
    }

    @Override
    public void loop() {
        computerVision.update();
        telemetry.addData("Spike: ", computerVision.checkSpike(isBlue));
        telemetry.addData("TF Recognitions: ", + computerVision.getTensorFlowRecognitions().size());
    }
}
