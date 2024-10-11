//import MecanumDrive;
//
//public class MyOpmode extends LinearOpMode {
//    @Override
//    public void runOpMode() {
//        MecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//
//        // EASTER EGG idk i didnt plan an easter egg
//        // ALL OF THE POSITIONS ARE ESTIMATES
//
//        // TEST
//        Trajectory myTrajectory = drive.trajectoryBuilder(new Pose2d)
//                .linetoX(24)
//                .splineTo(new Vector2d(48, 0), Math.PI / 2)
//                .build();
//
//
//        // So do we just copy paste this into the auton test that shryansh made?
//
//        // TELEOP SETUP STRAT
//        // Start near team specific samples
//
//        Pose2d startPose = new Pose2d(-40, -12, Math.toRadians(0));
//
//        // Go to samples
//        Trajectory Traj1 = drive.trajectoryBuilder(startPose)
//                //.splineToConstantHeading(sample pos estimate, - Math.PI / 2)
//                //.afterDisp(0, DispMarker) idk how but we pick up the sample here
//                .build();
//        // Push samples to observation zones
//        Trjectory Traj2 = drive.trajectoryBuilder(Traj1.end())
//                //.lineToY(65)
//                .build();
//
//        // AUTONOMUS FOLLOW STRAT
//        // Start near alliance neutral samples
//
//
//        waitForStart();
//
//        if(isStopRequested()) return;
//
//        drive.followTrajectory(myTrajectory);
//    }
//}