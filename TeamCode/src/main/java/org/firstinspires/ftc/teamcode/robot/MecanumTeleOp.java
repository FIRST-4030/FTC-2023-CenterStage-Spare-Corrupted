package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.drives.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.gamepad.gamepad.InputAutoMapper;
import org.firstinspires.ftc.teamcode.gamepad.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.general.Pose2dWrapper;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@TeleOp(name = "MecanumTeleOp")
public class MecanumTeleOp extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor LeftFront, LeftRear, RightRear, RightFront;
    //dog = arm
    DcMotor LF = LeftFront;
    DcMotor LR = LeftRear;
    DcMotor RR = RightRear;
    DcMotor RF = RightFront;

    InputHandler inputHandler;

    public Pose2dWrapper InitialPoint = new Pose2dWrapper(-37, -70, Math.toRadians(90.00));

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        LeftFront  = hardwareMap.get(DcMotor.class, "left_drive");
        RightFront = hardwareMap.get(DcMotor.class, "right_drive");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        LeftFront.setDirection(DcMotor.Direction.REVERSE);
        RightFront.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)

        inputHandler = InputAutoMapper.normal.autoMap(this);

        NewMecanumDrive drive = new NewMecanumDrive(hardwareMap);

        waitForStart();
        runtime.reset();




        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            if(gamepad1.x) {
                drive.setPoseEstimate(InitialPoint.toPose2d());

                TrajectorySequence InitialPoint = drive.trajectorySequenceBuilder(new Pose2d(-36.95, -69.35, Math.toRadians(90.00)))
                        .splineTo(new Vector2d(-43.39, -12.88), Math.toRadians(74.23))
                        .splineTo(new Vector2d(41.12, -21.22), Math.toRadians(2.93))
                        .splineTo(new Vector2d(41.31, 14.02), Math.toRadians(89.69))
                        .splineTo(new Vector2d(-28.42, -2.65), Math.toRadians(187.70))
                        .splineTo(new Vector2d(-61.20, -0.19), Math.toRadians(170.49))
                        .build();

            }

            // Setup a variable for each drive wheel to save power level for telemetry
            double LeftStrength;
            double RightStrength;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.

            //double drivingmode = -gamepad1.left_stick_y;
            //double turn  =  gamepad1.right_stick_x;
            //LeftStrength    = Range.clip(drive + turn, -1.0, 1.0) ;
            //RightStrength   = Range.clip(drive - turn, -1.0, 1.0) ;

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", LeftStrength, RightStrength);
            telemetry.addLine("hey we left left the rat in chagre");
            telemetry.update();
        }
    }
}
