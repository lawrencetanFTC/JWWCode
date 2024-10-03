import MecanumDrive

public class MyOpmode extends LinearOpMode {
    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        Trajectory myTrajectory = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(5)
                .forward(5)
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(myTrajectory);
    }
}