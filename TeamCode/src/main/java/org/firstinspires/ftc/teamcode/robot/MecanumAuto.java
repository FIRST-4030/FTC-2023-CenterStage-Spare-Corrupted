package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.drives.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.general.Pose2dWrapper;

@Autonomous(name = "MecanumAuto")
public class MecanumAuto extends LinearOpMode {

    // Declare OpMode members.
    //private ElapsedTime runtime = new ElapsedTime();

    private DcMotor LeftFront, LeftRear, RightRear, RightFront;
    //dog = arm
    DcMotor LF = LeftFront;
    DcMotor LR = LeftRear;
    DcMotor RR = RightRear;
    DcMotor RF = RightFront;
    //double LFStrength;
    //double RFStrength;
    //double LRStrength;
    //double RRStrength;
    //InputHandler inputHandler;
    //IMU imu;
    NewMecanumDrive drive;
    //Orientation or;
    public Pose2dWrapper InitialPoint = new Pose2dWrapper(-37, -70, Math.toRadians(90.00));



/*
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        )));
        imu.resetYaw();
        or = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYX, AngleUnit.RADIANS);
        drive = new NewMecanumDrive(hardwareMap);
*/
/*

    public void handleInput() {
        inputHandler.loop();
        if(inputHandler.down("D1:X")) {
            drive.setPoseEstimate(InitialPoint.toPose2d());

            TrajectorySequence InitialPoint = drive.trajectorySequenceBuilder(new Pose2d(-36.95, -69.35, Math.toRadians(90.00)))
                    .splineTo(new Vector2d(-43.39, -12.88), Math.toRadians(74.23))
                    .splineTo(new Vector2d(41.12, -21.22), Math.toRadians(2.93))
                    .splineTo(new Vector2d(41.31, 14.02), Math.toRadians(89.69))
                    .splineTo(new Vector2d(-28.42, -2.65), Math.toRadians(187.70))
                    .splineTo(new Vector2d(-61.20, -0.19), Math.toRadians(170.49))
                    .build();
            telemetry.addLine("yes it works this is a miracle");
            drive.followTrajectorySequence(InitialPoint);
            telemetry.addLine("THIS IS A CHAT MOMENT I REPEAT A CHAT MOMENT");
        }
        telemetry.addLine("chat 10 gifted chat");
        do {

        } while (opModeInInit());
    }
*/
    @Override
    public void runOpMode() {

        //handleInput();

        //inputHandler = InputAutoMapper.normal.autoMap(this);

        drive = new NewMecanumDrive(hardwareMap);

        LeftFront  = hardwareMap.get(DcMotor.class, "LF");
        RightFront = hardwareMap.get(DcMotor.class, "RF");
        RightRear = hardwareMap.get(DcMotor.class, "RR");
        LeftRear= hardwareMap.get(DcMotor.class, "LR");

        LeftFront.setDirection(DcMotor.Direction.REVERSE);
        RightFront.setDirection(DcMotor.Direction.FORWARD);
        LeftRear.setDirection(DcMotor.Direction.REVERSE);
        RightRear.setDirection(DcMotor.Direction.FORWARD);

        drive.setPoseEstimate(InitialPoint.toPose2d());

       //TrajectorySequence Fredrick = drive.trajectorySequenceBuilder(new Pose2d(-36.95, -69.35, Math.toRadians(90.00)))
        Trajectory fredrick = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineTo(new Vector2d(-43.39, -12.88), Math.toRadians(74.23))
                .splineTo(new Vector2d(41.12, -21.22), Math.toRadians(2.93))
                //.splineTo(new Vector2d(41.31, 14.02), Math.toRadians(89.69))
                //.splineTo(new Vector2d(-28.42, -2.65), Math.toRadians(187.70))
                //.splineTo(new Vector2d(-61.20, -0.19), Math.toRadians(170.49))
                .build();
        telemetry.addLine("yes it works this is a miracle");
        drive.followTrajectory(fredrick);
        telemetry.addLine("THIS IS A CHAT MOMENT I REPEAT A CHAT MOMENT");

        sleep(100000);
/*
        double drivingmode = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
        LFStrength   = Range.clip(drive + turn, -1.0, 1.0) ;
        RFStrength   = Range.clip(drive - turn, -1.0, 1.0) ;
 */
        telemetry.addLine("hey we left left the rat in chagre");
        telemetry.update();
    }
}
