package com.example.meepmeeptestingdos;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Action;



public class MeepMeepTesting {
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(800);
        int increment = 5;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(70, 70, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(15, 16.125)
                .build();

        RoadRunnerBotEntity myBot2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(80, 80, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(15, 16.125)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(15, -62.5, Math.toRadians(90)))
                 .strafeToLinearHeading(new Vector2d(0, -47), Math.toRadians(-90)) // Hang Specimen
                 .strafeToConstantHeading(new Vector2d(10, -47))
                 // .splineToConstantHeading(new Vector2d(36.50, -24.00), Math.toRadians(90.00))
                 .splineToConstantHeading(new Vector2d(40, 0.00), Math.toRadians(90.00))
                 .strafeToLinearHeading(new Vector2d(48, 0), Math.toRadians(90))
                 .strafeToConstantHeading(new Vector2d(48, -52.5))
                 .splineToConstantHeading(new Vector2d(58, 0), Math.toRadians(0.00))
                 .strafeToConstantHeading(new Vector2d(58, -49))
                 .turn(Math.toRadians(-180))
                 .strafeToConstantHeading(new Vector2d(0, -47))
                 .strafeToConstantHeading(new Vector2d(58, -49))
                 .strafeToConstantHeading(new Vector2d(2, -47))
                 .strafeToConstantHeading(new Vector2d(58, -49))
                 .strafeToConstantHeading(new Vector2d(-2, -47))
                 .build());

        myBot2.runAction(myBot2.getDrive().actionBuilder(new Pose2d(23.5, 62.5, -Math.PI/ 2))
                .strafeToConstantHeading(new Vector2d(0, 37))
                .strafeToConstantHeading(new Vector2d(5, 37))
                .splineToLinearHeading(new Pose2d(36.50, 24.00, Math.toRadians(90.00)), Math.toRadians(-90)) //
                .splineToConstantHeading(new Vector2d(48.00, 0.00), Math.toRadians(0.00))
                .strafeToLinearHeading(new Vector2d(42, 52.5), Math.toRadians(-135))
                .strafeToConstantHeading(new Vector2d(48, 52.5)) //
                .strafeToConstantHeading(new Vector2d(46.5, 52.5))
                .splineToLinearHeading(new Pose2d(58, 0, Math.toRadians(90.00)), Math.toRadians(0.00))
                .strafeToLinearHeading(new Vector2d(48, 52.5), Math.toRadians(-135))
//                .build()); // Bucket Case

                .strafeToConstantHeading(new Vector2d(0, 40))
                        .strafeToConstantHeading(new Vector2d(5, 40))
                        .splineToConstantHeading(new Vector2d(36.50, 24.00), Math.toRadians(-90.00))
                        .splineToConstantHeading(new Vector2d(48.00, 0.00), Math.toRadians(0.00))
                        .strafeToConstantHeading(new Vector2d(48, 52.5))
                        .splineToConstantHeading(new Vector2d(58, 0), Math.toRadians(0.00))
                        .strafeToConstantHeading(new Vector2d(58, 49)) // Push
                                .build());

//                .strafeToConstantHeading(new Vector2d(0, -35)) // Move to chamber
//                .splineToLinearHeading(new Pose2d(24, -24, Math.toRadians(90)), Math.toRadians(0)) // Move to first sample
//                .turn(Math.toRadians(-180)) // Sweep
//                .turnTo(Math.toRadians(45)) // Sweep back
//                .strafeToConstantHeading(new Vector2d(28, -24)) // Move to next sample
//                .turn(Math.toRadians(-180)) // Sweep next sample
//                .turnTo(Math.toRadians(45)) // Sweep back
//                .strafeToConstantHeading(new Vector2d(32, -24))
//                .turn(Math.toRadians(-180)) // Sweep next sample
//                .turnTo(Math.toRadians(90)) // Sweep back

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
//                .addEntity(myBot)
                .addEntity(myBot2)
                .start();
    }}