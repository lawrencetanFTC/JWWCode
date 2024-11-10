package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous
public class goForward  extends LinearOpMode {
    double power = .24;


    private DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
    private DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
    private DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
    private DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");

    MecanumDrive drive = new MecanumDrive( hardwareMap, new Pose2d(0,0 ,0) );




    public void runOpMode(){
        Pose2d InitialPose = new Pose2d(0, 0, 0);
        waitForStart();
        if (opModeIsActive()) {;

            DefaultMotorPower(frontLeft, backLeft, frontRight, backRight);
            Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                        .strafeToLinearHeading(new Vector2d(0,1), 0)
                        .build()
            );
            power = 0;
            DefaultMotorPower(frontLeft, backLeft, frontRight,backRight);


        }
    }

  public  void DefaultMotorPower(DcMotor frontLeft, DcMotor frontRight,DcMotor backLeft, DcMotor backRight){
          frontLeft.setPower(power);
          frontRight.setPower(power);
          backLeft.setPower(power);
          backRight.setPower(power);


    }}


