package org.firstinspires.ftc.teamcode.robot;

import static java.util.concurrent.TimeUnit.MILLISECONDS;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.drives.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector3d;
import org.firstinspires.ftc.teamcode.gamepad.gamepad.InputAutoMapper;
import org.firstinspires.ftc.teamcode.gamepad.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.util.LinearMotorController;

import java.util.HashMap;

@TeleOp
public class MecanumTeleOp extends OpMode {
    NewMecanumDrive drive;
    LinearMotorController liftController;
    LinearMotorController hookController;
    InputHandler inputHandler;
    Vector3d mecanumController;
    Servo armServo;

    Servo leftFlipper;
    Servo rightFlipper;
    Servo droneServo;
    DcMotorSimple intake;
    DcMotor hook;
    ElapsedTime flipperTime = new ElapsedTime();
    ElapsedTime droneTime = new ElapsedTime();
    ElapsedTime droneLimit = new ElapsedTime();

    double commandedPosition = 0.04;
    double minArmPos = 0.04;
    double maxArmPos = 0.275;
    boolean useFlipper = false;
    boolean launchDrone = false;
    boolean intakeRunning = false;
    double intakePower = 1;
    int currentLiftPos;
    boolean resetArm = false;
    double armPower;
    int hookMult = 0;

    //Create a hash map with keys: dpad buttons, and values: ints based on the corresponding joystick value of the dpad if is pressed and 0 if it is not
    //Ex. dpad Up = 1, dpad Down = -1
    //I chose to use a hashmap for human readability, even if it adds more lines of code, unsure if this was the correct choice but hey, I made it
    HashMap<String, Integer> dpadPowerMap = new HashMap<>();
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
        liftController = new LinearMotorController(hardwareMap, "Lift", 1200, true);
        hookController = new LinearMotorController(hardwareMap, "Hook", 3750, false);
        inputHandler = InputAutoMapper.normal.autoMap(this);

        //values for gamepad joystick values represented as a vector3D
        mecanumController = new Vector3d();

        //initialize arm
        armServo = hardwareMap.get(Servo.class, "Arm");

        //initialize intake
        intake = hardwareMap.get(DcMotorSimple.class, "Intake");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        //initialize flipper motors
        leftFlipper = hardwareMap.get(Servo.class, "leftHook");
        rightFlipper = hardwareMap.get(Servo.class, "rightHook");

        leftFlipper.setPosition(0.999);
        rightFlipper.setPosition(0.01);

        droneServo = hardwareMap.get(Servo.class, "Drone");
        droneServo.setPosition(0.3);
        droneLimit.reset();

    }

    @Override
    public void loop() {
        handleInput();
        drive.update(mecanumController, dpadPowerArray);
        liftController.update(gamepad2.right_stick_y, armServo.getPosition(), 7);
        hookController.update(hookMult, 19);
        armServo.setPosition(commandedPosition);
        telemetry.addData("armPos: ", commandedPosition);
        telemetry.addData("targetLiftPos: ", liftController.target);
        telemetry.addData("actualLiftPos: ", currentLiftPos);
        telemetry.addData("right_stick_y ", gamepad2.right_stick_y);
        telemetry.update();
    }

    public void handleInput() {
        inputHandler.loop();
        currentLiftPos = liftController.getLiftMotor().getCurrentPosition();


        //y values of sticks are inverted, thus minus
        armPower = gamepad2.left_stick_x;
        if(inputHandler.active("D2:DPAD_UP")){
            armPower = 1;
        }
        if(inputHandler.active("D2:DPAD_DOWN")){
            armPower = -1;
        }
        commandedPosition = commandedPosition + 0.00075 * armPower;

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

        if(currentLiftPos >= 10){
            resetArm = true;
        }
        if(currentLiftPos < 10 && armServo.getPosition() > minArmPos + 0.0001 && resetArm == true){
            commandedPosition = minArmPos;
            resetArm = false;
        }

        mecanumController = new Vector3d(gamepad1.left_stick_x , gamepad1.left_stick_y, gamepad1.right_stick_x);


        //Checks to see if the dpad is pressed, if it is replace 0 on the hashmap with the corresponding joystick value
        if(inputHandler.active("D1:DPAD_UP")) {
            dpadPowerMap.put("Up", 1);
        } else { dpadPowerMap.put("Up", 0); }

        if(inputHandler.active("D1:DPAD_DOWN")) {
            dpadPowerMap.put("Down", -1);
        } else { dpadPowerMap.put("Down", 0); }

        if(inputHandler.active("D1:DPAD_LEFT")) {
            dpadPowerMap.put("Left", -1);
        } else { dpadPowerMap.put("Left", 0); }

        if(inputHandler.active("D1:DPAD_RIGHT")) {
            dpadPowerMap.put("Right", 1);
        } else { dpadPowerMap.put("Right", 0); }

        dpadPowerArray = new int[]{dpadPowerMap.get("Up"), dpadPowerMap.get("Down"), dpadPowerMap.get("Left"), dpadPowerMap.get("Right")};
        telemetry.addData("Dpad Array: ", dpadPowerArray);



        if(inputHandler.up("D2:LB")) {
            intakeRunning = !intakeRunning;
        }
        //check if the intake should be running and
        if(intakeRunning && armServo.getPosition() <= minArmPos + 0.01){
            intake.setPower(intakePower);
        } else {intake.setPower(0); intakeRunning = false;}

        if(inputHandler.up("D2:RB")){
            intakePower = -1 * intakePower;
        }
        //set arm pos to the max position
        if(inputHandler.up("D2:Y")){
            commandedPosition = maxArmPos;
        }

        if(inputHandler.up("D2:LT")){
            useFlipper = true;
            flipperTime.reset();
        }
        if(useFlipper){
            leftFlipper.setPosition(0.4);
            rightFlipper.setPosition(0.6);
            if(flipperTime.milliseconds() > 650) {
                leftFlipper.setPosition(0.999);
                rightFlipper.setPosition(0.01);
                useFlipper = false;
            }
        }
        if(inputHandler.up("D1:RT") && droneLimit.seconds() > 85){
            launchDrone = true;
            droneTime.reset();
        }
        if(launchDrone){
            droneServo.setPosition(0.1);
            if(droneTime.milliseconds() > 450){
                droneServo.setPosition(0.3);
                launchDrone = false;
            }
        }
        if(inputHandler.active("D1:A")){
            hookMult = 1;
        }
        else if(inputHandler.active("D1:Y")){
            hookMult = -1;
        } else {
            hookMult = 0;
        }




    }

}
