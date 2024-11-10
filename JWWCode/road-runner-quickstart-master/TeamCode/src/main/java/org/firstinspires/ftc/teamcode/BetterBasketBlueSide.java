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
       1. Right Shoulder Motor
       2. Left Shoulder Motor
       3. Spintake Base Motor
       4. Back Motor
       5. Spintake Base Servo
       6. Spintake Servo

       THESE NAMES ARE NOT PERMANENT!!!!!!
       DOUBLE CHECK EVERY hardwareMap.get() !!!!!!!!!
     */

    // 1
    public class RShoulderMotor {
        private DcMotor RShoudler;
        
        public RShoulderMotor(HardwareMap hardwareMap) {
            RShoudler = hardwareMap.get(DcMotor.class, "RShoulderMotor");
            RShoudler.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            // Might cause problems, delete comment if works
            RShoudler.setDirection(DcMotor.Direction.BACKWARD);
        }
        // Actions go here
    }

    // 2
    public class LShoulderMotor {
        private DcMotor LShoudler;

        public LShoulderMotor(HardwareMap hardwareMap) {
            LShoudler = hardwareMap.get(DcMotor.class, "LShoulderMotor");
            LShoudler.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            // Might cause problems, delete comment if works
            LShoudler.setDirection(DcMotor.Direction.FORWARD);
        }
        // Actions go here
    }

    // 3
    public class SpintakeBaseMotor {
        private DcMotor SpintakeBase;

        public SpintakeBaseMotor(HardwareMap hardwareMap) {
            SpintakeBase = hardwareMap.get(DcMotor.class, "SpintakeBaseMotor");
            SpintakeBase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            // Might cause problems, delete comment if works
            SpintakeBase.setDirection(DcMotor.Direction.FORWARD);
        }
        // Actions go here
    }

    // 4
    public class BackMotor {
        private DcMotor BackM;

        public BackMotor(HardwareMap hardwareMap) {
            BackM = hardwareMap.get(DcMotor.class, "RshoulderMotor");
            BackM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            // Might cause problems, delete comment if works
            BackM.setDirection(DcMotor.Direction.BACKWARD);
        }
        // Actions go here
    }
    
    // 5
    public class SpintakeBaseMotor {
        private Servo SpintakeBase;

        public SpintakeBaseMotor(HardwareMap hardwareMap) {
            SpintakeBase = hardwareMap.get(Servo.class, "SpintakeBaseServo");
        }
    }

    // 6
    public class SpintakeServo {
        private Servo Spintake;

        public SpintakeServo(HardwareMap hardwareMap) {
            Spintake = hardwareMap.get(Servo.class, "SpintakeServo");
        }
    }

    @Override
    public void runOpMode() {
        // Starting pose (0, 0, 0)
        Pose2d InitialPose = new Pose2d(0, 0, 0);
        Pose2d secondPose = new Pose2d(-6, -24, 0);

        MecanumDrive drive = new MecanumDrive(hardwareMap, InitialPose);

        waitForStart();

        if (opModeIsActive()) {

            // Build all the Actions here

            // Move from Waypoint 1 (0, 0) to Waypoint 2 (-6, -24)
            TrajectoryActionBuilder OneToTwo = drive.actionBuilder(InitialPose)
                    .strafeTo(new Vector2d(-6, -24));

            // Move to Waypoint 3 (-48, -24)
            TrajectoryActionBuilder TwoToThree = drive.actionBuilder(secondPose)
                    .strafeTo(new Vector2d(-48, -24));

            // Repeat 3 times {
            // Move to Waypoint 4 (-48, -6)
            TrajectoryActionBuilder ThreeToFour = drive.actionBuilder(new Pose2d(-48, -24, Math.toRadians(0)))
                    .strafeToLinearHeading(new Vector2d(-48, -6), Math.toRadians(135));

            // Move back to Waypoint 3 (-48, -24)
            TrajectoryActionBuilder FourToThree = drive.actionBuilder(new Pose2d(-48, -6, Math.toRadians(135)))
                    .strafeToLinearHeading(new Vector2d(-48, -24), Math.toRadians(0));
            // }

            // Move back to Waypoint 2 (-6, -24) for TeleOp
            TrajectoryActionBuilder FourToTwo = drive.actionBuilder(new Pose2d(-48, -6, Math.toRadians(135)))
                    .strafeToLinearHeading(new Vector2d(-6, -24), Math.toRadians(0));

            // This should stop if you press stop
            if (isStopRequested()) return;

            Actions.runBlocking(
                    new SequentialAction(
                            OneToTwo.build(),
                            // Hang Specimen
                            TwoToThree.build(),
                            // Get Samples
                            ThreeToFour.build(),
                            // Push samples in Observation zone
                            FourToThree.build(),
                            // Get Samples again
                            ThreeToFour.build(),
                            // Push samples in Observation zone
                            FourToThree.build(),
                            // Get Samples once more
                            ThreeToFour.build(),
                            // Return to Waypoint 2 for TeleOp
                            FourToTwo.build()
                    )
            );
        }
    }
}