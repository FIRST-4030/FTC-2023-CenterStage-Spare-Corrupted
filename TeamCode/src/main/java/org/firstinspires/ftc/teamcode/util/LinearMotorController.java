package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LinearMotorController {
    DcMotor liftMotor;
    public int target;
    public int tickLimit;
    public LinearMotorController(HardwareMap hardwareMap, String liftMotorName, int maxVal, boolean reverse){
        liftMotor = hardwareMap.get(DcMotor.class, liftMotorName);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setPower(1);
        if(reverse) {liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);}
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tickLimit = maxVal;
    }

    public void update(double motorControl, double armPos){
        if(armPos >= 0.07) {
            target = target - (int) Math.round(motorControl * 5);
            if (target < 1) {
                target = 1;
            } else if (target > tickLimit) {
                target = tickLimit;
            }
            liftMotor.setTargetPosition(target);
        }

    }
    public void update(double motorControl){
            target = target - (int) Math.round(motorControl * 5);
            if (target < 1) {
                target = 1;
            } else if (target > tickLimit) {
                target = tickLimit;
            }
            liftMotor.setTargetPosition(target);

    }
    public void setTarget(int targetPos, double armPos){
        if(armPos >= 0.07) {
            liftMotor.setTargetPosition(targetPos);
            target = targetPos;
        }
    }
    public void setTarget(int targetPos){
            liftMotor.setTargetPosition(targetPos);
            target = targetPos;
    }
    public DcMotor getLiftMotor(){
        return liftMotor;
    }
}
