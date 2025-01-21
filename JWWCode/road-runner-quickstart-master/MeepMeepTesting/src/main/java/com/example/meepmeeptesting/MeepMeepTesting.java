package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

// edit
public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity myBot2 = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(65, 65, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(17.5, 17)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(15.35, -63.5 , Math.toRadians(90))) //TODO Fix starting Pos
//                .splineToSplineHeading(new Pose2d(52, 52, Math.toRadians(45.00)), Math.toRadians(45.00))
//                .strafeToLinearHeading(new Vector2d(48,52), Math.toRadians(-90))
//                .strafeToLinearHeading(new Vector2d(52,52), Math.toRadians(45.00))
//                .strafeToLinearHeading(new Vector2d(58,45), Math.toRadians(-90))
//                .strafeToLinearHeading(new Vector2d(52,52), Math.toRadians(45.00))
                .strafeToConstantHeading(new Vector2d(2, -45))
                .splineToConstantHeading(new Vector2d(27.00, -37.00), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(47.00, -10.00), Math.toRadians(0))
                .strafeToConstantHeading(new Vector2d(47.00, -45.00))
                .splineToConstantHeading(new Vector2d(47.00, -20.00), Math.toRadians(78.65))
                .splineToConstantHeading(new Vector2d(56.50, -12.00), Math.toRadians(27.55))
                .strafeToConstantHeading(new Vector2d(56.50, -45.00))
                .splineToConstantHeading(new Vector2d(42.90, -44.03), Math.toRadians(196.93))
                .splineToLinearHeading(new Pose2d(27.00, -58.00, Math.toRadians(0.00)), Math.toRadians(173.67))
                .strafeToConstantHeading(new Vector2d(32.00, -58.00))
                .splineToLinearHeading(new Pose2d(1.60, -35.95, Math.toRadians(90.00)), Math.toRadians(144.05))
                .splineToConstantHeading(new Vector2d(3.66, -46.65), Math.toRadians(-67.14))
                .splineToLinearHeading(new Pose2d(26.94, -57.92, Math.toRadians(0.00)), Math.toRadians(224.46))
                .splineToLinearHeading(new Pose2d(1.60, -35.95, Math.toRadians(90.00)), Math.toRadians(144.05))


//                .strafeTo(new Vector2d(48, 48))
//                .splineTo(new Vector2d(53.00, 53.00), Math.toRadians(45))
//
//                .turn(Math.toRadians(-135))
//                .strafeTo(new Vector2d(58,51))
//
//                .strafeTo(new Vector2d(58, 48))
//                .splineTo(new Vector2d(53.00, 53.00), Math.toRadians(45))
//
//                .turn(Math.toRadians(-135))
//                .splineToLinearHeading(new Pose2d(47,27, Math.toRadians(0)), Math.toRadians(0))

//                .strafeToLinearHeading(new Vector2d(48, 27), Math.toRadians(0))
//                .strafeToLinearHeading(new Vector2d(53.00, 53.00), Math.toRadians(45))
//                //.splineTo(new Vector2d(52.00, 45.00), Math.toRadians(238.03))
//                .splineToLinearHeading(new Pose2d(25.00, 5, Math.toRadians(180.00)), Math.toRadians(150.00))

//                .splineTo(new Vector2d(52.00, 45.00), Math.toRadians(238.03))
//                .splineToLinearHeading(new Pose2d(25.00, 13.00, Math.toRadians(180.00)), Math.toRadians(180.00))
                .build());


        myBot2.runAction(myBot2.getDrive().actionBuilder(new Pose2d(15,62.5, Math.toRadians(-90.00)))
                .splineTo(new Vector2d(53.00, 53.00), Math.toRadians(45))
                .turn(Math.toRadians(-135))
                .strafeTo(new Vector2d(48, 51))
                .strafeTo(new Vector2d(48, 48))
                .splineTo(new Vector2d(53.00, 53.00), Math.toRadians(45))
                .turn(Math.toRadians(-135))
                .strafeTo(new Vector2d(58,51))
                .strafeTo(new Vector2d(58, 48))
                .splineTo(new Vector2d(53.00, 53.00), Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(47,27, Math.toRadians(0)), Math.toRadians(0))
                .strafeTo(new Vector2d(48, 27))
                .splineTo(new Vector2d(53.00, 53.00), Math.toRadians(45))
                .splineTo(new Vector2d(52.00, 45.00), Math.toRadians(238.03))
                .splineToLinearHeading(new Pose2d(25.00, 13.00, Math.toRadians(180.00)), Math.toRadians(180.00))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)

                .addEntity(myBot)
                .start();
    }
}