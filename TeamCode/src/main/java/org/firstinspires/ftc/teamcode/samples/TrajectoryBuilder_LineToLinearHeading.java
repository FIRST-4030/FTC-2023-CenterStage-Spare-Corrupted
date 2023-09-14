package org.firstinspires.ftc.teamcode.samples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utils.roadrunnerUtils.DashboardUtil;
import org.firstinspires.ftc.teamcode.general.Pose2dWrapper;

/*
 * Robot moves to the specified coordinates while linearly
 * interpolating between the start heading and a specified end heading.
 */
@Config
@Autonomous(group = "drive")
public class TrajectoryBuilder_LineToLinearHeading extends LinearOpMode {
    public static double END_X = 40; // in
    public static double END_Y = 50; // in
    public static double HEADING = 90; // in

    public Pose2dWrapper startPose = new Pose2dWrapper(0, 0, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose.toPose2d());

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d( END_X, END_Y, Math.toRadians(HEADING)))
                .build();

        DashboardUtil.previewTrajectories(FtcDashboard.getInstance(), traj1);

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(traj1);
    }
}