package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.robot.ComputerVision;

import java.util.List;

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
        //telemetryTfod();
        telemetry.addData("Spike: ", computerVision.checkSpike(isBlue));
       // telemetry.addData("TF Recognitions: ", + computerVision.getTensorFlowRecognitions().size());
       // if(computerVision.getTensorFlowRecognitions().size() > 0) {
      //      telemetry.addData("left pos: ", computerVision.getTensorFlowRecognitions().get(0).getLeft());
      //  }
    }
    private void telemetryTfod() {

        List<Recognition> currentRecognitions = computerVision.getTensorFlowRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }
}
