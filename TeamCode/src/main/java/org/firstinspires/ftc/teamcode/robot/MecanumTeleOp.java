package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM;
import static java.util.concurrent.TimeUnit.MILLISECONDS;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.drives.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector3d;
import org.firstinspires.ftc.teamcode.gamepad.gamepad.InputAutoMapper;
import org.firstinspires.ftc.teamcode.gamepad.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.util.LinearMotorController;
import org.firstinspires.ftc.teamcode.drive.RobotConstants;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.HashMap;

@TeleOp
public class MecanumTeleOp extends OpMode {
    NewMecanumDrive drive;
    LinearMotorController liftController;
    LinearMotorController hookController;
    DistanceSensor secondPixelDetector;
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
    ElapsedTime headingTimer = new ElapsedTime();

    double commandedPosition = 0.04;
    double minArmPos = 0.04;
    double maxArmPos = 0.275;
    double dpadPower = 1;
    boolean useFlipper = false;
    boolean launchDrone = false;
    boolean intakeRunning = false;
    boolean override = false;
    boolean resetHeading = false;
    double intakePower = 1;
    int currentLiftPos;
    boolean resetArm = false;
    double armPower;
    int hookMult = 0;
    double driveCoefficient;
    IMU imu;
    Orientation or;
    IMU.Parameters myIMUparameters;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime armMoveTimer = new ElapsedTime();
    double deltaTime;
    double previousTime;

    double globalIMUHeading;
    double headingError = 0;
    boolean resetIMU = false;

    //Create a hash map with keys: dpad buttons, and values: ints based on the corresponding joystick value of the dpad if is pressed and 0 if it is not
    //Ex. dpad Up = 1, dpad Down = -1
    //I chose to use a hashmap for human readability, even if it adds more lines of code, unsure if this was the correct choice but hey, I made it
    HashMap<String, Double> dpadPowerMap = new HashMap<>();
    double[] dpadPowerArray = new double[4];
    double[] savedAngles = new double[]{0, 0, 0};
    double powerCoefficient = 1;
    boolean precisionDrive = false;
    double currentDist;
    double[] distArray = new double[15];
    int distIterator = 0;
    double distAverage = 0;



    @Override
    public void init() {
        //init dpad hashmap with each dpad value as unpressed
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters( new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        )));
        imu.resetDeviceConfigurationForOpMode();
        or = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
        globalIMUHeading = or.thirdAngle;

        dpadPowerMap.put("Up", 0.0);
        dpadPowerMap.put("Down", 0.0);
        dpadPowerMap.put("Left", 0.0);
        dpadPowerMap.put("Right", 0.0);


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
        secondPixelDetector = hardwareMap.get(DistanceSensor.class, "detector");;

        //initialize flipper motors
        leftFlipper = hardwareMap.get(Servo.class, "leftHook");
        rightFlipper = hardwareMap.get(Servo.class, "rightHook");

        leftFlipper.setPosition(0.999);
        rightFlipper.setPosition(0.01);

        droneServo = hardwareMap.get(Servo.class, "Drone");
        droneServo.setPosition(0.3);
        droneLimit.reset();
        timer.reset();
        previousTime = 0;

    }

    @Override
    public void loop() {
        deltaTime = timer.milliseconds() - previousTime;
        previousTime += deltaTime;
        telemetry.addData("deltatime: ", deltaTime);
        currentDist = secondPixelDetector.getDistance(MM);
        distArray[distIterator % 14] = currentDist;
        distIterator++;
        for(int i = 0; i < distArray.length; i++){
            distAverage += distArray[i];
        }
        distAverage /= 15;
        handleInput();
        outputLog();
        or = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
        headingError = or.thirdAngle - globalIMUHeading;
        telemetry.addData("error: ", headingError);
        resetIMU = drive.update(mecanumController, dpadPowerArray, headingError, resetIMU, powerCoefficient);
        liftController.update(gamepad2.right_stick_y, armServo.getPosition(), (int) Math.round( 1.2*deltaTime));
        hookController.update(hookMult, (int)Math.round(1.7*deltaTime));
        armServo.setPosition(commandedPosition);
        telemetry.addData("current distance: ", currentDist);
        telemetry.addData("current distance average: ", distAverage);
        telemetry.addData("armPos: ", commandedPosition);
        telemetry.addData("targetLiftPos: ", liftController.target);
        telemetry.addData("actualLiftPos: ", currentLiftPos);
        telemetry.addData("right_stick_y ", gamepad2.right_stick_y);
        telemetry.addData("override ", override);
        telemetry.addData("Robot angle 1: ", or.firstAngle);
        telemetry.addData("Robot angle 2: ", or.secondAngle);
        telemetry.addData("Robot angle 3: ", or.thirdAngle);
        telemetry.update();
    }

    public void handleInput() {
        inputHandler.loop();
        currentLiftPos = liftController.getLiftMotor().getCurrentPosition();
        if(hookController.target > 10){
            driveCoefficient = 0.3;
        } else {
            driveCoefficient = 1;
        }


        //y values of sticks are inverted, thus minus
        armPower = gamepad2.left_stick_x;
        if(inputHandler.active("D2:DPAD_UP")){
            armPower = 1;
        }
        if(inputHandler.active("D2:DPAD_DOWN")){
            armPower = -1;
        }
        commandedPosition = commandedPosition + 0.00012 * armPower * deltaTime;

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

        if(gamepad1.right_stick_x != 0 && headingTimer.milliseconds() > 100){
            resetHeading = true;
            headingTimer.reset();
        }
        if(resetHeading){
            if(headingTimer.milliseconds() > 175){
                globalIMUHeading = or.thirdAngle;
                resetHeading = false;
            }
        }
        mecanumController = new Vector3d((gamepad1.left_stick_x * driveCoefficient), (gamepad1.left_stick_y * driveCoefficient), (gamepad1.right_stick_x * driveCoefficient));


        //Checks to see if the dpad is pressed, if it is replace 0 on the hashmap with the corresponding joystick value
        if(hookController.target > 10){
            dpadPower = 0.3;
        }
        if(inputHandler.active("D1:DPAD_UP")) {
            dpadPowerMap.put("Up", dpadPower);
        } else { dpadPowerMap.put("Up", 0.0); }

        if(inputHandler.active("D1:DPAD_DOWN")) {
            dpadPowerMap.put("Down", -dpadPower);
        } else { dpadPowerMap.put("Down", 0.0); }

        if(inputHandler.active("D1:DPAD_LEFT")) {
            dpadPowerMap.put("Left", -dpadPower);
        } else { dpadPowerMap.put("Left", 0.0); }

        if(inputHandler.active("D1:DPAD_RIGHT")) {
            dpadPowerMap.put("Right", dpadPower);
        } else { dpadPowerMap.put("Right", 0.0); }

        dpadPowerArray = new double[]{dpadPowerMap.get("Up"), dpadPowerMap.get("Down"), dpadPowerMap.get("Left"), dpadPowerMap.get("Right")};
        telemetry.addData("Dpad Array: ", dpadPowerArray);



        if(inputHandler.up("D2:LB")) {
            intakeRunning = !intakeRunning;
        }
        //check if the intake should be running and
        if(intakeRunning && armServo.getPosition() <= minArmPos + 0.01 && distAverage > 39){
            intake.setPower(intakePower);
        } else {intake.setPower(0); intakeRunning = false;}

        if(inputHandler.up("D2:RB")){
            intakePower = -1 * intakePower;
        }
        //set arm pos to the max position
        if(inputHandler.up("D2:Y")) {
            commandedPosition = maxArmPos;
        }
        /*if(resetArm){
            if(armMoveTimer.milliseconds() > 1850){
                commandedPosition = 0.220;
                resetArm = false;
            }
        }*/
        if(inputHandler.up("D2:X")){
            commandedPosition = minArmPos;
            armMoveTimer.reset();
            resetArm = true;
        }
        if(resetArm && armMoveTimer.milliseconds() > 400){
            liftController.setTarget(1);
            resetArm = false;
        }
        if(inputHandler.up("D2:GUIDE")){
            override = !override;
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
        if(inputHandler.up("D1:B")){
            resetIMU = true;
        }
        if(inputHandler.up("D1:RT") && true /*(droneLimit.seconds() > 85 || override)*/){
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
        if(inputHandler.up("D1:R1")){
            precisionDrive = !precisionDrive;
        }
        if(precisionDrive){
            powerCoefficient = 0.5;
        } else {
            powerCoefficient = 1;
        }




    }
    public void outputLog(){
        if(Math.abs(or.firstAngle) > Math.abs(savedAngles[0]) + 0.005 || Math.abs(or.secondAngle) > Math.abs(savedAngles[1]) + 0.005 || Math.abs(or.thirdAngle) > Math.abs(savedAngles[2]) + 0.005) {
            RobotLog.d("WAY: IMU Angles = %.03f, %.03f, %.03f, %.03f", or.firstAngle, or.secondAngle, or.thirdAngle, previousTime);
        }
    }

}
