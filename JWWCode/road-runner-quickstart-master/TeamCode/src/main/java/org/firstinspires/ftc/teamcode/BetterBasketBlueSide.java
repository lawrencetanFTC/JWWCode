package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Pose2d;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "BetterDeckAuto", group = "Autonomous")
public class BetterBasketBlueSide extends LinearOpMode {

    /* REMEMBER TO DELETE THE ONES WE DON'T NEED!!
        Index of Non-drivetrain Motors and Servos
       1. RP Servo
       2. Spintake Servo
       3. Claw Servo
       4. Shoulder Servo
       5. Claw Motor
       6. Shoulder Motor
       7. Back Motor (ignore??)

       THESE NAMES ARE NOT PERMANENT!!!!!!
     */

    // 1
    public class RPServo {
        private Servo rpservo;

        public RPServo(HardwareMap hardwareMap) {
            // CHANGE LATER
            rpservo = hardwareMap.get(Servo.class, "rpservo");
        }
        // Actions Go here
    }

    // 2
    public static class SpintakeServo {
        private Servo spntkservo;

        public SpintakeServo(HardwareMap hardwareMap) {
            // CHANGE LATER
            spntkservo = hardwareMap.get(Servo.class, "spntkservo");
        }
        // Actions Go here
    }

    // 3
    public static class ClawServo {
        private Servo claw;

        public ClawServo(HardwareMap hardwareMap) {
            // CHANGE LATER
            claw = hardwareMap.get(Servo.class, "clawservo");
        }
        // Actions Go here
    }

    // 4
    public static class ShoulderServo {
        private Servo shoulder;

        public ShoulderServo(HardwareMap hardwareMap) {
            // CHANGE LATER
            shoulder = hardwareMap.get(Servo.class, "shoulderservo");
        }
        // Actions Go here
    }

    // 5
    public static class ClawMotor {
        // Might be something else Do check later
        private DcMotor clawmotor;

        public ClawMotor(HardwareMap hardwareMap) {
            clawmotor = hardwareMap.get(DcMotor.class, "clawmotor");
            clawmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            // Might cause problem, delete comment if works
            clawmotor.setDirection(DcMotor.Direction.FORWARD);
        }
        // Actions Go here
    }

    // 5
    public static class ShoulderMotor {
        // Might be something else Do check later
        private DcMotor shouldermotor;

        public ShoulderMotor(HardwareMap hardwareMap) {
            shouldermotor = hardwareMap.get(DcMotor.class, "clawmotor");
            shouldermotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            // Might cause problem, delete comment if works
            shouldermotor.setDirection(DcMotor.Direction.FORWARD);
        }
        // Actions Go here
    }


    @Override
    public void runOpMode() {
        // Starting pose (0, 0, 0)
        Pose2d InitialPose = new Pose2d(0, 0, 0);

        MecanumDrive drive = new MecanumDrive(hardwareMap, InitialPose);

        waitForStart();

        if (opModeIsActive()) {

            // All Actions defined here

            // Move from Waypoint 1 (0, 0) to Waypoint 2 (-6, -24)
            Action OneToTwo = drive.actionBuilder(InitialPose)
                    .strafeTo(new Vector2d(-6, -24))
                    .build();

            // Move to Waypoint 3 (-48, -24)
            Action TwoToThree = drive.actionBuilder(-6, -24)
                    .strafeTo(new Vector2d(-48, -24))
                    .build();

            // Repeat 3 times {
            // Move to Waypoint 4 (-48, -6)
            Action ThreeToFour = drive.actionBuilder(new Pose2d(-48, -24, Math.toRadians(0)))
                    .strafeToLinearHeading(new Vector2d(-48, -6), Math.toRadians(135))
                    .build();

            // Move back to Waypoint 3 (-48, -24)
            Action FourToThree = drive.actionBuilder(new Pose2d(-48, -6, Math.toRadians(135)))
                    .strafeToLinearHeading(new Vector2d(-48, -24), Math.toRadians(0))
                    .build();
            // }

            // Move back to Waypoint 2 (-6, -24) for TeleOp
            Action FourToTwo = drive.actionBuilder(new Pose2d(-48, -6, Math.toRadians(135)))
                    .strafeToLinearHeading(new Vector2d(-6, -24), Math.toRadians(0))
                    .build();

            // This should stop if you press stop
            if (isStopRequested()) return;

            Actions.runBlocking(
                    new SequentialAction(
                            OneToTwo,
                            // Hang Specimen
                            // Get Sample
                            TwoToThree,
                            // Get Samples
                            ThreeToFour,
                            // Push samples in Observation zone
                            FourToThree,
                            // Get Samples again
                            ThreeToFour,
                            // Push samples in Observation zone
                            FourToThree,
                            // Get Samples once more
                            ThreeToFour,
                            // Return to Waypoint 2 for TeleOp
                            FourToTwo
                    )
            );
        }
    }
}