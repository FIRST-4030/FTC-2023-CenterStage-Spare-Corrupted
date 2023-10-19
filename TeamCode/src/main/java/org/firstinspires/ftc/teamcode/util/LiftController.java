package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftController {
    DcMotor liftMotor;
    public int target;
    public LiftController(HardwareMap hardwareMap, String liftMotorName){
        liftMotor = hardwareMap.get(DcMotor.class, liftMotorName);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setPower(0.5);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void update(double liftControl, double armPos){
        if(armPos >= 0.07) {
            target = target - (int) Math.round(liftControl * 5);
            if (target < 1) {
                target = 1;
            } else if (target > 1200) {
                target = 1200;
            }
            liftMotor.setTargetPosition(target);
        }
    }
    public void setTarget(int targetPos, double armPos){
        if(armPos >= 0.07) {
            liftMotor.setTargetPosition(targetPos);
            target = targetPos;
        }
    }
}
