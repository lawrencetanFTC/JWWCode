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
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(17.5, 17)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(26.35, 58.5 , Math.toRadians(0))) //TODO Fix starting Pos
                .splineToSplineHeading(new Pose2d(52, 52, Math.toRadians(45.00)), Math.toRadians(45.00))
                .strafeToLinearHeading(new Vector2d(48,52), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(52,52), Math.toRadians(45.00))
                .strafeToLinearHeading(new Vector2d(58,45), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(52,52), Math.toRadians(45.00))

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

                .strafeToLinearHeading(new Vector2d(48, 27), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(53.00, 53.00), Math.toRadians(45))

//                .splineTo(new Vector2d(52.00, 45.00), Math.toRadians(238.03))
//                .splineToLinearHeading(new Pose2d(25.00, 13.00, Math.toRadians(180.00)), Math.toRadians(180.00))
                .build());


        myBot2.runAction(myBot2.getDrive().actionBuilder(new Pose2d(0,37, Math.toRadians(-90.00)))
                .strafeTo(new Vector2d(-31, 37))
                .turn(Math.toRadians(90))
                .strafeTo(new Vector2d(-31,10))
                .strafeTo(new Vector2d(-41, 10))
                .strafeTo(new Vector2d(-41,54))
                .strafeTo(new Vector2d(-41,10))
                .strafeTo(new Vector2d(-53,10))
                .strafeTo(new Vector2d(-53, 54))
                .strafeTo(new Vector2d(-53,45))
                .splineTo(new Vector2d(-40.09, 46.26), Math.toRadians(-30.94))
                .splineToLinearHeading(new Pose2d(-23.00, 59.03, Math.toRadians(180.00)), Math.toRadians(180.00))
                .splineTo(new Vector2d(-41.17, 47.09), Math.toRadians(7.31))
                .splineTo(new Vector2d(-30.19, 46.87), Math.toRadians(2.10))
                .splineTo(new Vector2d(-1.69, 47.51), Math.toRadians(45.00))
                .splineToLinearHeading(new Pose2d(0.00, 37.00, Math.toRadians(-90.00)), Math.toRadians(-90.00))

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .addEntity(myBot2)
                .start();
    }
}