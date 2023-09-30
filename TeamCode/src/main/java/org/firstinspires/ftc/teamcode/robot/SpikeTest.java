package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.general.Pose2dWrapper;
import org.firstinspires.ftc.teamcode.drive.CustomMecanumDrive;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.Endpoint;

@Autonomous(name = "SpikeTest")
public class SpikeTest extends LinearOpMode {
    public Pose2dWrapper startPose = new Pose2dWrapper(16, -64, Math.toRadians(90));
    public boolean red = true;
    public int spike;
    double spikePointX = 16;
    double spikePointY = -32;
    double heading = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        CustomMecanumDrive drive = new CustomMecanumDrive(hardwareMap, 1, 1, 1);


        drive.setPoseEstimate(startPose.toPose2d());

        /*
        if (red) {
            spikePointX = 0;
            spikePointY = 0;
            switch (spike) {
                case 0:
                    heading = 0;
                    break;
                case 1:
                    heading = 1;
                    break;
                case 2:
                    heading = 2;
                    break;
            }

        } else {
            spikePointX = 0;
            spikePointY = 0;
            switch (spike) {
                case 0:
                    heading = 0;
                    break;
                case 1:
                    heading = 1;
                    break;
                case 2:
                    heading = 2;
                    break;
            }
        }
        */


        Endpoint spikePoint = new Endpoint(spikePointX, spikePointY, heading);



        Trajectory traj1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .splineTo(new Vector2d(22, -40), Math.toRadians(45))
                                .build();

        DashboardUtil.previewTrajectories(FtcDashboard.getInstance(), traj1);

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(traj1);

    }
}
