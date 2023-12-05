package com.example.meepmeep7462;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeep7462 {

    static RoadRunnerBotEntity myBot = null;

    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(900);

        boolean isBlue     = false;
        boolean isAudience = false;
        int spike          = 3;

        if (isBlue) {
            if (isAudience) {
                blueAudience(meepMeep, spike);

            } else {
                blueNoAudience(meepMeep, spike);
            }
        } else {
            if (isAudience) {
                redAudience(meepMeep, spike);
            } else {
                redNoAudience(meepMeep, spike);
            }
        }

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

    static void redAudience(MeepMeep inMeep, int inSpike) {
        int maxVel = 30, maxAccel = 30;
        double trackWidth = 15;
        int spikeHeading = 0, startHeading = 90;

        double startX           =  -39,      startY           = -62.5;
        double spikeX           =   0,       spikeY           =     0;
        double mediaryPoseX     =  -(15+34), mediaryPoseY     = -50.5, mediaryPoseHeading = 180;
        double backdropPoseX    = 47.75,     backdropPoseY     = -35.5;
        double pixelPoseX       = -59.75,    pixelPoseY        = -36.5;
        double centerPoseX      = -58,       centerPoseY       = -7.5;
        double travelPoseX      = 35,        travelPoseY       = -8.5;
        double postPixelPoseX   = -59.75,    postPixelPoseY    = -36.5;
        double tempParkPoseX    = 48,        tempParkPoseY     = -13.5;
        double aprilTagPoseX    = 50,        aprilTagY         = -37;
        double outerTravelPoseX = 50, outerTravelPoseY  = -37;
//    double aprilTagPoseX = 50, aprilTagY      = -37;
//    double aprilTagPoseX = 50, aprilTagY      = -37;
//

        switch (inSpike) {
            case 1:
                spikeX       = 3.7-47;
                spikeY       = -34.5;
                spikeHeading = 115;
                aprilTagY    = -28.3;
                break;
            case 2:
                spikeX       = 11-47;
                spikeY       = -35.5;
                spikeHeading = 90;
                aprilTagY    = -35.5;
                break;
            case 3:
                spikeX       = 17.5-47;
                spikeY       = -34.5;
                spikeHeading = 65;
                aprilTagY    = -42.4;
                break;
        }

        Pose2dWrapper startPose = new Pose2dWrapper(startX, startY, Math.toRadians(startHeading));
        Pose2dWrapper spikePose = new Pose2dWrapper(spikeX, spikeY, spikeHeading);
        Pose2dWrapper mediaryPose = new Pose2dWrapper(mediaryPoseX, mediaryPoseY, 180);
        Pose2dWrapper pixelPose = new Pose2dWrapper(pixelPoseX, pixelPoseY, 0);
        Pose2dWrapper centerPose = new Pose2dWrapper(centerPoseX, centerPoseY, 0);
        Pose2dWrapper travelPose = new Pose2dWrapper(travelPoseX, travelPoseY, 0);
        Pose2dWrapper postPixelPose = new Pose2dWrapper(postPixelPoseX, postPixelPoseY, 0);
        Pose2dWrapper backdropPose = new Pose2dWrapper(backdropPoseX, backdropPoseY, 0);
        Pose2dWrapper aprilTagPose = new Pose2dWrapper(aprilTagPoseX, aprilTagY, 0);
        Pose2dWrapper tempParkPose = new Pose2dWrapper(tempParkPoseX, tempParkPoseY, 0);

        myBot = new DefaultBotBuilder(inMeep)
                .setConstraints(maxVel, maxAccel, Math.toRadians(180), Math.toRadians(180), trackWidth)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(startPose.x,startPose.y,startPose.heading))
                                .splineTo(spikePose.toPose2d().vec(), Math.toRadians(spikePose.heading))
                                .setReversed(true)
                                .splineTo(mediaryPose.toPose2d().vec(), Math.toRadians(mediaryPoseHeading))
                                .strafeTo(pixelPose.toPose2d().vec())
                                .strafeTo(centerPose.toPose2d().vec())
                                .strafeTo(travelPose.toPose2d().vec())
//                                .lineToLinearHeading(postPixelPose.toPose2d())
//                                .splineTo(backdropPose.toPose2d().vec(), Math.toRadians(backdropPose.heading))
                                .setReversed(true)
//                                .splineTo(aprilTagPose.toPose2d().vec(), Math.toRadians(aprilTagPose.heading))
//                                .strafeTo(tempParkPose.toPose2d().vec())
                                .build() );    }

    static void redNoAudience(MeepMeep inMeep, int inSpike) {

        int maxVel = 30, maxAccel = 30;
        double trackWidth = 15;
        int spikeHeading = 0, startHeading = 90;

        double startX        =  15, startY       = -62.5;
        double spikeX        =   0, spikeY       =     0;
        double mediaryPoseX  =  15, mediaryPoseY = -50.5, mediaryPoseHeading = 0;
        double backdropPoseX = 36, backdropPoseY = -35.5;
        double outerTravelPoseX = 24, outerTravelPoseY     = -58;
        double outerCenterPoseX = -40, outerCenterY     = -58;
        double postPixelPoseX       = -59.75,    postPixelPoseY        = -36.5;
        double aprilTagPoseX = 50, aprilTagY     = -37;
        double tempParkPoseX = 48, tempParkPoseY = -61.5;

        switch (inSpike) {
            case 1:
                spikeX       = 3.7;
                spikeY       = -34.5;
                spikeHeading = 115;
                aprilTagY    = -28.3;
                break;
            case 2:
                spikeX       = 11;
                spikeY       = -35.5;
                spikeHeading = 90;
                aprilTagY    = -35.5;
                break;
            case 3:
                spikeX       = 17.5;
                spikeY       = -34.5;
                spikeHeading = 65;
                aprilTagY    = -42.4;
                break;
        }

        Pose2dWrapper startPose = new Pose2dWrapper(15, -62.5, Math.toRadians(90));
        Pose2dWrapper spikePose = new Pose2dWrapper(spikeX, spikeY, spikeHeading);
        Pose2dWrapper mediaryPose = new Pose2dWrapper(15, -50.5, 0);
        Pose2dWrapper audiencePose = new Pose2dWrapper(-58, -41.5, 0);
        Pose2dWrapper backdropPose = new Pose2dWrapper(36, -37.5, 0);
        Pose2dWrapper centerPose = new Pose2dWrapper(-58, -7.5, 0);
        Pose2dWrapper outerCenterPose = new Pose2dWrapper(-48, -61, 0);
        Pose2dWrapper tempParkPose = new Pose2dWrapper(48, -61.5, 0);
        Pose2dWrapper travelPose = new Pose2dWrapper(35, -8.5, 0);
        Pose2dWrapper outerTravelPose = new Pose2dWrapper(12, -61, 0);
        Pose2dWrapper aprilTagPose = new Pose2dWrapper(50, aprilTagY, 0);
        Pose2dWrapper pixelPose = new Pose2dWrapper(-58, -36.5, 0);
        Pose2dWrapper postPixelPose = new Pose2dWrapper(-59.75, -36.5, 0);
        Pose2dWrapper avoidancePose = new Pose2dWrapper(-59 , -36.5, 0);

        myBot = new DefaultBotBuilder(inMeep)
                .setConstraints(maxVel, maxAccel, Math.toRadians(180), Math.toRadians(180), trackWidth)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(startPose.x,startPose.y,startPose.heading))
                                .splineTo(spikePose.toPose2d().vec(), Math.toRadians(spikePose.heading))
                                .setReversed(true)
                                .splineTo(mediaryPose.toPose2d().vec(), Math.toRadians(180-0))
                                .setReversed(false)
                                .splineTo(backdropPose.toPose2d().vec(), Math.toRadians(backdropPose.heading))
                                .strafeTo(aprilTagPose.toPose2d().vec())
                                .splineToConstantHeading(outerTravelPose.toPose2d().vec(), Math.toRadians(outerTravelPose.heading))
//                                .splineToConstantHeading(outerCenterPose.toPose2d().vec(), Math.toRadians(outerCenterPose.heading))
//                                .strafeTo(pixelPose.toPose2d().vec())
//                                .splineToConstantHeading(outerTravelPose.toPose2d().vec(), Math.toRadians(outerTravelPose.heading))
//                                .splineTo(backdropPose.toPose2d().vec(), Math.toRadians(backdropPose.heading))
//                                .strafeTo(aprilTagPose.toPose2d().vec())
//                                .strafeTo(tempParkPose.toPose2d().vec())

//                                .lineToLinearHeading(postPixelPose.toPose2d().vec(),)
//                                .strafeTo(outerCenterPose.toPose2d().vec())
//                                .strafeTo(aprilTagPose.toPose2d().vec())
                                .build() );
    }
    static void blueAudience(MeepMeep inMeep, int inSpike) {
    }

    static void blueNoAudience(MeepMeep inMeep, int inSpike) {
    }
}
