package com.example.meepmeep7462;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTester {

    static RoadRunnerBotEntity myBot = null;

    public static void main(String[] args) {

        final MeepMeep meepMeep = new MeepMeep(900);

        myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 13.45 )
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-37, -70, Math.toRadians(90.00)))
                            .splineTo(new Vector2d(-45, 11), Math.toRadians(-5.25))
                            .lineToConstantHeading(new Vector2d(-13, -12))
                            .splineTo(new Vector2d(14, -14), Math.toRadians(22.36))
                            .splineTo(new Vector2d(38, -4), Math.toRadians(57.09))
                            .splineTo(new Vector2d(17, 15), Math.toRadians(180.00))
                            .lineToSplineHeading(new Pose2d(-58, 15, Math.toRadians(90.00)))
                            .build() );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
