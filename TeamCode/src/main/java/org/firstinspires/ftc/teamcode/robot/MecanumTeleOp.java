package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.sun.source.doctree.SerialFieldTree;

import org.firstinspires.ftc.teamcode.drive.drives.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector3d;
import org.firstinspires.ftc.teamcode.gamepad.gamepad.InputAutoMapper;
import org.firstinspires.ftc.teamcode.gamepad.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.util.LiftController;

@TeleOp
public class MecanumTeleOp extends OpMode {
    NewMecanumDrive drive;
    LiftController liftController;
    InputHandler inputHandler;
    Vector3d controller;
    Servo armServo;
    DcMotorSimple intake;

    double commandedPosition = 0.13;
    double minServoPos = 0.135;
    double maxServoPos = 0.4;
    boolean intakeRunning = false;
    double intakePower = 1;
    @Override
    public void init() {
        drive = new NewMecanumDrive(hardwareMap);
        liftController = new LiftController(hardwareMap, "Lift");
        inputHandler = InputAutoMapper.normal.autoMap(this);
        controller = new Vector3d();
        armServo = hardwareMap.get(Servo.class, "Arm");
        intake = hardwareMap.get(DcMotorSimple.class, "Intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {
        handleInput();
        drive.update(controller);
        liftController.update(gamepad2.right_stick_y);
        armServo.setPosition(commandedPosition);
        telemetry.addData("armPos: ", commandedPosition);
        telemetry.addData("liftPos: ", liftController.target);
        telemetry.addData("right_stick_y ", gamepad2.right_stick_y);
        telemetry.update();
    }

    public void handleInput() {
        inputHandler.loop();
        //y values of sticks are inverted, thus minus
        commandedPosition = commandedPosition + 0.00075 * gamepad2.left_stick_x;
        if (commandedPosition < minServoPos) {
            commandedPosition = minServoPos;
        }
        if (commandedPosition > maxServoPos) {
            commandedPosition = maxServoPos;
        }
        controller = new Vector3d(gamepad1.left_stick_x , gamepad1.left_stick_y, gamepad1.right_stick_x);

        if(inputHandler.up("D1:LB")) {
            intakeRunning = !intakeRunning;
        }
        if(intakeRunning){
            intake.setPower(intakePower);
        } else {intake.setPower(0);}

        if(inputHandler.up("D1:RB")){
            intakePower = -1 * intakePower;
        }

        if(inputHandler.up("D2:Y")){
            commandedPosition = maxServoPos;
        }




    }

}
