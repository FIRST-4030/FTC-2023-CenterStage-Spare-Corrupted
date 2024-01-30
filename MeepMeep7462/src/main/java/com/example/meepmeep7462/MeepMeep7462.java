package com.example.meepmeep7462;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeep7462 {

    static RoadRunnerBotEntity myBot = null;

    public static void main(String[] args) {

        final MeepMeep meepMeep = new MeepMeep(900);

        /*
        ** Edit the configuration ffo MeepMeep7462 to set the following 3 values
         */
        final boolean isBlue     = Boolean.parseBoolean(System.getenv("isBlue"));
        final boolean isAudience = Boolean.parseBoolean(System.getenv("isAudience"));;
        final int     thisSpike  = Integer.parseInt(System.getenv("spike"));

        final FieldSide fieldSide = new FieldSide( isBlue, isAudience, thisSpike);

        final Pose pose = new Pose();

        final Spike spike = new Spike( pose, fieldSide);

        if (fieldSide.isAudience) {

            final Pose2dWrapper startPose     = new Pose2dWrapper(spike.startPose.x,    spike.startPose.y,    Math.toRadians(spike.startPose.heading));
            final Pose2dWrapper spikePose     = new Pose2dWrapper(spike.spikePoint.x,   spike.spikePoint.y,   spike.spikePoint.heading);
            final Pose2dWrapper mediaryPose   = new Pose2dWrapper(pose.mediaryPose.x,   pose.mediaryPose.y,   pose.mediaryPose.heading);
            final Pose2dWrapper pixelPose     = new Pose2dWrapper(pose.pixelPose.x,     pose.pixelPose.y,     pose.pixelPose.heading);
            final Pose2dWrapper avoidancePose = new Pose2dWrapper(pose.avoidancePose.x, pose.avoidancePose.y, pose.avoidancePose.heading);
            final Pose2dWrapper centerPose    = new Pose2dWrapper(pose.centerPose.x,    pose.centerPose.y,    pose.centerPose.heading);
            final Pose2dWrapper travelPose    = new Pose2dWrapper(pose.travelPose.x,    pose.travelPose.y,    pose.travelPose.heading);
            final Pose2dWrapper postPixelPose = new Pose2dWrapper(pose.postPixelPose.x, pose.postPixelPose.y, pose.postPixelPose.heading);
            final Pose2dWrapper aprilTagPose  = new Pose2dWrapper(pose.aprilTagPose.x,  pose.aprilTagPose.y,  pose.aprilTagPose.heading);
            final Pose2dWrapper tempParkPose  = new Pose2dWrapper(pose.tempParkPose.x,  pose.tempParkPose.y,  pose.tempParkPose.heading);

            myBot = new DefaultBotBuilder(meepMeep)
                    .setConstraints(fieldSide.maxVel, fieldSide.maxAccel, Math.toRadians(180), Math.toRadians(180), fieldSide.trackWidth)
                    .followTrajectorySequence(drive ->
                            drive.trajectorySequenceBuilder(new Pose2d(startPose.x,startPose.y,startPose.heading))
                                    .splineTo(spikePose.toPose2d().vec(), Math.toRadians(spikePose.heading))
                                    .setReversed(true)
                                    .splineTo(mediaryPose.toPose2d().vec(), Math.toRadians(180-pose.mediaryPose.heading))
                                    .strafeTo(pixelPose.toPose2d().vec())
                                    .lineToConstantHeading(postPixelPose.toPose2d().vec())
                                    .strafeTo(avoidancePose.toPose2d().vec())
                                    .strafeTo(centerPose.toPose2d().vec())
                                    .strafeTo(travelPose.toPose2d().vec())
                                    .strafeTo(aprilTagPose.toPose2d().vec())
                                    .strafeTo(tempParkPose.toPose2d().vec())
                                    .build() );

        } else {
            final Pose2dWrapper startPose       = new Pose2dWrapper(spike.startPose.x,      spike.startPose.y,      Math.toRadians(spike.startPose.heading));
            final Pose2dWrapper spikePose       = new Pose2dWrapper(spike.spikePoint.x,     spike.spikePoint.y,     spike.spikePoint.heading);
            final Pose2dWrapper mediaryPose     = new Pose2dWrapper(pose.mediaryPose.x,     pose.mediaryPose.y,     pose.mediaryPose.heading);
            final Pose2dWrapper backdropPose    = new Pose2dWrapper(pose.backdropPose.x,    pose.backdropPose.y,    pose.backdropPose.heading);
            final Pose2dWrapper outerTravelPose = new Pose2dWrapper(pose.outerTravelPose.x, pose.outerTravelPose.y, pose.outerTravelPose.heading);
            final Pose2dWrapper outerCenterPose = new Pose2dWrapper(pose.outerCenterPose.x, pose.outerCenterPose.y, pose.outerCenterPose.heading);
            final Pose2dWrapper tempParkPose    = new Pose2dWrapper(pose.tempParkPose.x,    pose.tempParkPose.y,    pose.tempParkPose.heading);
            final Pose2dWrapper aprilTagPose    = new Pose2dWrapper(pose.aprilTagPose.x,    pose.aprilTagPose.y,    pose.aprilTagPose.heading);
            final Pose2dWrapper pixelPose       = new Pose2dWrapper(pose.pixelPose.x,       pose.pixelPose.y,       pose.pixelPose.heading);
            final Pose2dWrapper postPixelPose   = new Pose2dWrapper(pose.postPixelPose.x,   pose.postPixelPose.y,   pose.postPixelPose.heading);

            myBot = new DefaultBotBuilder(meepMeep)
                    .setConstraints( fieldSide.maxVel, fieldSide.maxAccel, Math.toRadians(180), Math.toRadians(180), fieldSide.trackWidth )
                    .followTrajectorySequence(drive ->
                            drive.trajectorySequenceBuilder(new Pose2d(startPose.x,startPose.y,startPose.heading))
                                    .splineTo(spikePose.toPose2d().vec(), Math.toRadians(spikePose.heading))
                                    .setReversed(true)
                                    .splineTo(mediaryPose.toPose2d().vec(), Math.toRadians(180))
                                    .setReversed(false)
                                    .strafeTo(backdropPose.toPose2d().vec())
                                    .strafeTo(aprilTagPose.toPose2d().vec())
                                    .strafeTo(outerTravelPose.toPose2d().vec())
                                    .strafeTo(outerCenterPose.toPose2d().vec())
                                    .strafeTo(pixelPose.toPose2d().vec())
                                    .lineToConstantHeading(postPixelPose.toPose2d().vec())
                                    .waitSeconds(1)
                                    .splineToConstantHeading(outerCenterPose.toPose2d().vec(), Math.toRadians(outerCenterPose.heading))
                                    .splineToConstantHeading(outerTravelPose.toPose2d().vec(), Math.toRadians(outerTravelPose.heading))
                                    .splineToConstantHeading(backdropPose.toPose2d().vec(), Math.toRadians(backdropPose.heading))
                                    .strafeTo(aprilTagPose.toPose2d().vec())
                                    .strafeTo(tempParkPose.toPose2d().vec())
                                    .build() );
        }


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
