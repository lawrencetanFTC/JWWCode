
// May be useless remember to delete

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "BetterBasketRedSide", group = "Autonomous")
public class BetterBasketRedSide extends LinearOpMode {

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

