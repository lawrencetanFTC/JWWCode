//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.acmerobotics.roadrunner.DualNum; // Import DualNum for handling rotational data
//import com.acmerobotics.roadrunner.Vector2dDual; // Import Vector2dDual for 2D positional data
//
//@Autonomous(name="Real-Time Position Tracking", group="FTC")
//public class RealTimePositionTrackingOpMode extends LinearOpMode {
//
//    private TwoDeadWheelLocalizer localizer;
//    private double x = 0.0;  // Initial x position
//    private double y = 0.0;  // Initial y position
//    private double heading = 0.0;  // Initial heading (in radians)
//
//    @Override
//    public void runOpMode() {
//        // Initialize hardware and sensors
//        HardwareMap hardwareMap = this.hardwareMap;
//        IMU imu = hardwareMap.get(IMU.class, "imu");
//
//        // Setup encoders for the dead wheels (ensure your motor names match!)
//        DcMotorEx parMotor = hardwareMap.get(DcMotorEx.class, "par");
//        DcMotorEx perpMotor = hardwareMap.get(DcMotorEx.class, "perp");
//
//        // Initialize the localizer (assuming `inPerTick` is provided)
//        localizer = new TwoDeadWheelLocalizer(hardwareMap, imu, 1.0);  // You can adjust inPerTick
//
//        // Calibrate the robot or perform any setup necessary
//        parMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        perpMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//
//        // Wait for the start signal
//        waitForStart();
//
//        // Run the loop while the opMode is active
//        while (opModeIsActive()) {
//            // Update the position using the localizer
//            TwoDeadWheelLocalizer.Twist2dDual twist = localizer.update();
//
//            // Access position change (Vector2dDual)
//            Vector2dDual positionChange = twist.getTranslation(); // Use getTranslation()
//            double dx = positionChange.x.getReal();  // Get the real value (position change in x)
//            double dy = positionChange.y.getReal();  // Get the real value (position change in y)
//
//            // Access the heading change (DualNum)
//            DualNum headingChange = twist.getRotation(); // Use getRotation()
//            double dHeading = headingChange.getReal();  // Get the real value (change in heading)
//
//            // Integrate to update the position and heading
//            x += dx;
//            y += dy;
//            heading += dHeading;
//
//            // Log position and heading for telemetry
//            telemetry.addData("Position", "x: %.2f, y: %.2f, heading: %.2f", x, y, heading);
//            telemetry.update();
//
//            // Optionally, add control logic for autonomous actions
//            // Example: Move forward using the position data
//
//            // Pause to simulate a control loop (e.g., 20ms)
//            sleep(20);
//        }
//    }
//}
