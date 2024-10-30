
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

@Autonomous(name = "BetterDeckAuto", group = "Autonomous")
public class BetterDeckRedSide extends LinearOpMode {

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
    public class SpintakeServo {
        private Servo spntkservo;

        public SpintakeServo(HardwareMap hardwareMap) {
            // CHANGE LATER
            spntkservo = hardwareMap.get(Servo.class, "spntkservo");
        }
        // Actions Go here
    }

    // 3
    public class ClawServo {
        private Servo claw;

        public ClawServo(HardwareMap hardwareMap) {
            // CHANGE LATER
            claw = hardwareMap.get(Servo.class, "clawservo");
        }
        // Actions Go here
    }

    // 4
    public class ShoulderServo {
        private Servo shoulder;

        public ShoulderServo(HardwareMap hardwareMap) {
            // CHANGE LATER
            shoulder = hardwareMap.get(Servo.class, "shoulderservo");
        }
        // Actions Go here
    }

    // 5
    public class ClawMotor {
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
    public class ShoulderMotor {
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
        // Starting pose for the red side (mirrored coordinates)
        Pose2d InitialPose = new Pose2d(0, 0, 0);

        MecanumDrive drive = new MecanumDrive(hardwareMap, InitialPose);

        waitForStart();

        if (opModeIsActive()) {

            // Actions for the red side

            // Move from Waypoint 1 (0, 0) to Waypoint 2 (6, 24)
            TrajectoryActionBuilder OneToTwo = drive.actionBuilder(InitialPose)
                    .strafeTo(new Vector2d(6, 24));

            // Move to Waypoint 3 (48, 19)
            TrajectoryActionBuilder TwoToThree = drive.actionBuilder(new Pose2d(6, 24, Math.toRadians(0)))
                    .strafeTo(new Vector2d(48, 19));

            // Repeat 3 times
            // Move to Waypoint 4 (48, 19) with a turn to 180 degrees
            TrajectoryActionBuilder ThreeToFour = drive.actionBuilder(new Pose2d(48, 19, Math.toRadians(0)))
                    .turnTo(Math.toRadians(180));

            // Move back to Waypoint 3 (48, 19) with a turn to 0 degrees
            TrajectoryActionBuilder FourToThree = drive.actionBuilder(new Pose2d(48, 19, Math.toRadians(180)))
                    .turnTo(Math.toRadians(0));

            // Move back to Waypoint 2 (-6, 24) for TeleOp
            TrajectoryActionBuilder FourToTwo = drive.actionBuilder(new Pose2d(48, 19, Math.toRadians(180)))
                    .strafeTo(new Vector2d(-6, 24));

            // This should stop if you press stop
            if (isStopRequested()) return;

            Actions.runBlocking(
                    new SequentialAction(
                            OneToTwo.build(),
                            // Deposit Specimen
                            TwoToThree.build(),
                            // Pick up Sample
                            ThreeToFour.build(),
                            // Deposit Sample
                            FourToThree.build(),
                            // Pick up Sample
                            ThreeToFour.build(),
                            // Deposit Sample
                            FourToThree.build(),
                            // Pick up Sample
                            ThreeToFour.build(),
                            // Deposit Sample
                            FourToTwo.build()
                    )
            );
        }
    }
}
