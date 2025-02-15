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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-32.5, -63, Math.toRadians(-90.00)))
                .strafeToConstantHeading(new Vector2d(-32.5, -57))
                .splineToLinearHeading(new Pose2d(-53.5, -54, Math.toRadians(45.00)), Math.toRadians(-135.00))
                .splineToLinearHeading(new Pose2d(-48, -50, Math.toRadians(90.00)), Math.toRadians(90.00))
                .splineToLinearHeading(new Pose2d(-53.5, -54, Math.toRadians(45.00)), Math.toRadians(-135.00))
                .splineToLinearHeading(new Pose2d(-58.5, -49.5, Math.toRadians(90.00)), Math.toRadians(90.00))
                .splineToLinearHeading(new Pose2d(-53.5, -54, Math.toRadians(45.00)), Math.toRadians(-135.00))
                .splineToLinearHeading(new Pose2d(-23, -12, Math.toRadians(180.00)), Math.toRadians(-45.00))



                /*.strafeToConstantHeading(new Vector2d(0, -47)) // Hang Specimen
                 .strafeToConstantHeading(new Vector2d(10, -47))
                 // .splineToConstantHeading(new Vector2d(36.50, -24.00), Math.toRadians(90.00))
                 .splineToConstantHeading(new Vector2d(40, 0.00), Math.toRadians(90.00))
                 .strafeToConstantHeading(new Vector2d(48, 0))
                 .strafeToConstantHeading(new Vector2d(48, -52.5))
                 .splineToConstantHeading(new Vector2d(58, 0), Math.toRadians(0.00))
                 .strafeToConstantHeading(new Vector2d(58, -49))
                 .turn(Math.toRadians(-180))
                 .strafeToLinearHeading(new Vector2d(0, -47), Math.toRadians(90))
                 .strafeToLinearHeading(new Vector2d(58, -49), Math.toRadians(-90))
                 .strafeToLinearHeading(new Vector2d(0, -47), Math.toRadians(90))
                 .strafeToLinearHeading(new Vector2d(58, -49), Math.toRadians(-90))
                 .strafeToLinearHeading(new Vector2d(0, -47), Math.toRadians(90))*/
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
                .addEntity(myBot)
                .start();
    }}