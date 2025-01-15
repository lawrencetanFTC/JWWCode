package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity myBot2 = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(7.17, -34.00, Math.toRadians(90.00)))
                .strafeToConstantHeading(new Vector2d(37.00, -34.00))
                .splineToConstantHeading(new Vector2d(37.72, -18.91), Math.toRadians(64.50))
                .splineToConstantHeading(new Vector2d(48.16, -11.27), Math.toRadians(78.26))
                .strafeToConstantHeading(new Vector2d(48.00, -56.00))
                .splineToConstantHeading(new Vector2d(48.90, -21.52), Math.toRadians(75.57))
                .splineToConstantHeading(new Vector2d(58.21, -5.12), Math.toRadians(78.65))
                .strafeToConstantHeading(new Vector2d(58.40, -56.54))
                .splineToSplineHeading(new Pose2d(63.24, -26.73, Math.toRadians(0.00)), Math.toRadians(72.99))
                .build());



        myBot2.runAction(myBot2.getDrive().actionBuilder(new Pose2d(-2.4,35.45, Math.toRadians(-90.00)))
                .splineTo(new Vector2d(-22.57, 48.29), Math.toRadians(211.48))
                .splineTo(new Vector2d(-48.29, 9.43), Math.toRadians(199.24))
                .turn(Math.PI/2)
                .splineTo(new Vector2d(-49.43, 21.43), Math.toRadians(79.66))
                .splineTo(new Vector2d(-50.57, 58.00), Math.toRadians(92.69))
                .splineTo(new Vector2d(-47.71, 3.14), Math.toRadians(-87.02))
                .splineTo(new Vector2d(-57.71, 19.71), Math.toRadians(127.37))
                .splineTo(new Vector2d(-56.86, 31.43), Math.toRadians(76.33))
                .splineTo(new Vector2d(-57.14, 41.14), Math.toRadians(79.76))
                .splineTo(new Vector2d(-58.57, 64.00), Math.toRadians(91.11))
                .splineTo(new Vector2d(-68.57, -0.29), Math.toRadians(261.16))
                .splineTo(new Vector2d(-67.43, 64.00), Math.toRadians(88.98))

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot2)
                .start();
    }
}