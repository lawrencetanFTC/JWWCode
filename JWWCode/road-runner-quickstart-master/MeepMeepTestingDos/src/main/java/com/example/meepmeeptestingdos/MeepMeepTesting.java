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
                .setConstraints(65, 65, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(17.5, 17)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(13.5, -62.5, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(0, -47)) // Hang Specimen
                .strafeToLinearHeading(new Vector2d(24, -47), Math.toRadians(60)) // Move
                .turn(Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d((24 + (2*increment)), (-47 + (2*increment))), Math.toRadians(60)) // Increment move
                .turn(Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d((24 + (3*increment)), (-47 + (3*increment))), Math.toRadians(60)) // Increment move
                .turn(Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(55,-55), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(0, -47), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(55,-55), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(0, -47), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(55,-55), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(0, -47), Math.toRadians(90))
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