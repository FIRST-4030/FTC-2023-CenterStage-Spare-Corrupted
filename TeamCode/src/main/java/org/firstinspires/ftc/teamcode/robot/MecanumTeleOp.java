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

    Servo leftHook;
    Servo rightHook;
    DcMotorSimple intake;

    double commandedPosition = 0.135;
    double minArmPos = 0.135;
    double maxArmPos = 0.4;
    double minHookPos = 0.15;
    double maxHookPos = 0.575;
    boolean useHook = false;
    boolean intakeRunning = false;
    double intakePower = 1;
    @Override
    public void init() {
        //initialize drive
        drive = new NewMecanumDrive(hardwareMap);

        //initialize lift, gamepad handle
        liftController = new LiftController(hardwareMap, "Lift");
        inputHandler = InputAutoMapper.normal.autoMap(this);

        //values for gamepad joystick values represented as a vector3D
        controller = new Vector3d();

        //initialize arm
        armServo = hardwareMap.get(Servo.class, "Arm");

        //initialize intake
        intake = hardwareMap.get(DcMotorSimple.class, "Intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        //initialize hook motors
        leftHook = hardwareMap.get(Servo.class, "leftHook");
        rightHook = hardwareMap.get(Servo.class, "rightHook");

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
        if (commandedPosition < minArmPos) {
            commandedPosition = minArmPos;
        }
        if (commandedPosition > maxArmPos) {
            commandedPosition = maxArmPos;
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
            commandedPosition = maxArmPos;
        }

        if(inputHandler.up("D1:LT")){
            useHook = !useHook;
        }
        if(useHook){
            leftHook.setPosition(maxHookPos);
            rightHook.setPosition(maxHookPos);
        } else {
            leftHook.setPosition(minHookPos);
            rightHook.setPosition(minHookPos);
        }





    }

}
