package org.firstinspires.ftc.teamcode.utils.roadrunnerUtils;

//import com.acmerobotics.dashboard.canvas.Canvas;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import java.util.List;

/**
 * Set of helper functions for drawing Road Runner paths and trajectories on dashboard canvases.
 */
@Config
public class DashboardUtil {
    private static double DEFAULT_RESOLUTION = 2.0; // distance units; presumed inches
    private static double ROBOT_RADIUS = 9; // in

    private static String POSE_COLOR = "#FF9600FF"; //light orange
    private static String POSE_DIR_COLOR = "#FF3232FF"; //red
    private static String POSE_NORMAL_COLOR = "#43ff64FF"; //light green

    private static String TRAJECTORY_PREVIEW_COLOR = "#00CDFFFF"; //light blue

    /**
     * This method draws the given positions (given by poses) onto the field of the dashboard
     * @param canvas
     * @param poseHistory
     */
    public static void drawPoseHistory(Canvas canvas, List<Pose2d> poseHistory) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];
        for (int i = 0; i < poseHistory.size(); i++) {
            Pose2d pose = poseHistory.get(i);
            xPoints[i] = pose.getX();
            yPoints[i] = pose.getY();
        }
        canvas.strokePolyline(xPoints, yPoints);
    }

    /**
     * This method draws a given path onto the field in the dashboard
     * @param canvas
     * @param path
     * @param resolution
     */
    public static void drawSampledPath(Canvas canvas, Path path, double resolution) {
        int samples = (int) Math.ceil(path.length() / resolution);
        double[] xPoints = new double[samples];
        double[] yPoints = new double[samples];
        double dx = path.length() / (samples - 1);
        for (int i = 0; i < samples; i++) {
            double displacement = i * dx;
            Pose2d pose = path.get(displacement);
            xPoints[i] = pose.getX();
            yPoints[i] = pose.getY();
        }
        canvas.strokePolyline(xPoints, yPoints);
    }

    /**
     * This method draws a given path onto the field in the dashboard
     * @param canvas
     * @param path
     */
    public static void drawSampledPath(Canvas canvas, Path path) {
        drawSampledPath(canvas, path, DEFAULT_RESOLUTION);
    }

    /**
     * This method draws a virtual representation onto the field in the dashboard
     * @param canvas
     * @param pose
     */
    public static void drawRobot(Canvas canvas, Pose2d pose) {
        canvas.strokeCircle(pose.getX(), pose.getY(), ROBOT_RADIUS);
        Vector2d v = pose.headingVec().times(ROBOT_RADIUS);
        double x1 = pose.getX() + v.getX() / 2, y1 = pose.getY() + v.getY() / 2;
        double x2 = pose.getX() + v.getX(), y2 = pose.getY() + v.getY();
        canvas.strokeLine(x1, y1, x2, y2);
    }

    /**
     * This method draws a pose onto the field in the dashboard
     * @param canvas
     * @param pose2d
     */
    public static void drawPose(Canvas canvas, Pose2d pose2d){
        double originX = pose2d.getX(), originY = pose2d.getY();
        Vector2d dir = pose2d.headingVec().times(ROBOT_RADIUS);
        Vector2d normal = new Vector2d(dir.getY(), dir.getX() * -1);

        canvas.setStroke(POSE_COLOR);
        canvas.strokeCircle(originX, originY, ROBOT_RADIUS);

        canvas.setStroke(POSE_DIR_COLOR);
        canvas.strokeLine(originX, originY, originX + dir.getX(), originY + dir.getY());

        canvas.setStroke(POSE_NORMAL_COLOR);
        canvas.strokeLine(originX, originY, originX + normal.getX(), originY + normal.getY());
    }

    /**
     * This method is a wrapper method to {@code drawSampledPath()} that accepts a trajectory
     * @param canvas
     * @param trajectory
     */
    public static void drawTrajectory(Canvas canvas, Trajectory trajectory){
        drawSampledPath(canvas, trajectory.getPath());
    }

    /**
     * This method draws the path the trajectory makes and adds start and end poses onto the paths
     * @param dashboard
     * @param trajectories
     */
    public static void previewTrajectories(FtcDashboard dashboard, Trajectory... trajectories){
        TelemetryPacket packet = new TelemetryPacket();
        Canvas canvas = packet.fieldOverlay();
        for (Trajectory traj: trajectories) {
            canvas.setStroke(TRAJECTORY_PREVIEW_COLOR);
            drawTrajectory(canvas, traj);
            drawPose(canvas, traj.start());
            drawPose(canvas, traj.end());
        }
        dashboard.sendTelemetryPacket(packet);
    }
}