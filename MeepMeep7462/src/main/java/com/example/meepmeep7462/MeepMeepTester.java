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
                        drive.trajectorySequenceBuilder(new Pose2d(-36.95, -69.35, Math.toRadians(90.00)))
                                .splineTo(new Vector2d(-43.39, -12.88), Math.toRadians(74.23))
                                .splineTo(new Vector2d(41.12, -21.22), Math.toRadians(2.93))
                                .splineTo(new Vector2d(41.31, 14.02), Math.toRadians(89.69))
                                .splineTo(new Vector2d(-28.42, -2.65), Math.toRadians(187.70))
                                .splineTo(new Vector2d(-61.20, -0.19), Math.toRadians(170.49))
                                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
