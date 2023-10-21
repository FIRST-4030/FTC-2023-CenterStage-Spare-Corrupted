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

import java.util.HashMap;

@TeleOp
public class MecanumTeleOp extends OpMode {
    NewMecanumDrive drive;
    LiftController liftController;
    InputHandler inputHandler;
    Vector3d mecanumController;
    Servo armServo;

    Servo leftHook;
    Servo rightHook;
    DcMotorSimple intake;

    double commandedPosition = 0.04;
    double minArmPos = 0.04;
    double maxArmPos = 0.29;
    boolean useHook = false;
    boolean intakeRunning = false;
    double intakePower = 1;

    //Create a hash map with keys: dpad buttons, and values: booleans based on whether the button is pressed
    HashMap<String, Integer> dpadPowerMap = new HashMap<String, Integer>();
    int[] dpadPowerArray = new int[4];



    @Override
    public void init() {
        //init dpad hashmap with each dpad value as unpressed
        dpadPowerMap.put("Up", 0);
        dpadPowerMap.put("Down", 0);
        dpadPowerMap.put("Left", 0);
        dpadPowerMap.put("Right", 0);

        //initialize drive
        drive = new NewMecanumDrive(hardwareMap);

        //initialize lift, gamepad handle
        liftController = new LiftController(hardwareMap, "Lift");
        inputHandler = InputAutoMapper.normal.autoMap(this);

        //values for gamepad joystick values represented as a vector3D
        mecanumController = new Vector3d();

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
        drive.update(mecanumController, dpadPowerArray);
        liftController.update(gamepad2.right_stick_y, armServo.getPosition());
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

        if(gamepad2.right_stick_y < -0.05 && armServo.getPosition() < 0.07){
            commandedPosition = 0.071;
        }
        if(gamepad2.right_stick_y > 0.05 && armServo.getPosition() <= 0.07){
            commandedPosition = 0.071;
        }


        if (commandedPosition < minArmPos) {
            commandedPosition = minArmPos;
        }
        if (commandedPosition > maxArmPos) {
            commandedPosition = maxArmPos;
        }

        mecanumController = new Vector3d(gamepad1.left_stick_x , gamepad1.left_stick_y, gamepad1.right_stick_x);

        if(inputHandler.held("D1:DPAD_UP")) {
            dpadPowerMap.put("Up", 1);
        } else { dpadPowerMap.put("Up", 0); }

        if(inputHandler.held("D1:DPAD_DOWN")) {
            dpadPowerMap.put("Down", -1);
        } else { dpadPowerMap.put("Down", 0); }

        if(inputHandler.held("D1:DPAD_LEFT")) {
            dpadPowerMap.put("Left", -1);
        } else { dpadPowerMap.put("left", 0); }

        if(inputHandler.held("D1:DPAD_RIGHT")) {
            dpadPowerMap.put("Right", 1);
        } else { dpadPowerMap.put("Right", 0); }

        dpadPowerArray = new int[]{dpadPowerMap.get("Up"), dpadPowerMap.get("Down"), dpadPowerMap.get("Left"), dpadPowerMap.get("Right")};



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
        if(inputHandler.up("D2:X")){
            commandedPosition = 0.04;
            liftController.setTarget(1, armServo.getPosition());
        }

        if(inputHandler.up("D1:LT")){
            useHook = !useHook;
        }
        if(useHook){
            leftHook.setPosition(0.4);
            rightHook.setPosition(0.6);
            if(leftHook.getPosition() == 0.4){
                leftHook.setPosition(0.999);
                rightHook.setPosition(0.01);
                useHook = false;
            }
        }





    }

}
