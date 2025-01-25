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


        myBot2.runAction(myBot2.getDrive().actionBuilder(new Pose2d(15,61, Math.toRadians(-90.00)))
                .strafeToConstantHeading(new Vector2d(0, 47))
                .strafeToConstantHeading(new Vector2d(5, 47))
                .splineToConstantHeading(new Vector2d(36.50, 24.00), Math.toRadians(-90.00))
                .splineToConstantHeading(new Vector2d(48.00, 0.00), Math.toRadians(0.00))
                .strafeToConstantHeading(new Vector2d(48,52.5))
                .splineToConstantHeading(new Vector2d(58,0), Math.toRadians(0.00))
                .strafeToConstantHeading(new Vector2d(58,49))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)

                .addEntity(myBot2)
                .start();
    }
}