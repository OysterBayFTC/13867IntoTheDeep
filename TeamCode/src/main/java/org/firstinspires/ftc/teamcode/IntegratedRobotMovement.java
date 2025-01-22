// package org.firstinspires.ftc.teamcode.Base;
// 
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
// import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
// import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
// import java.util.Locale;
// 
// @TeleOp(name="Integrated Auto Movement", group="Linear OpMode")
// public class IntegratedRobotMovement extends LinearOpMode {
// 
//     // Declare hardware variables
//     private GoBildaPinpointDriver odo;
//     private DcMotor motorFrontLeft;
//     private DcMotor motorFrontRight;
//     private DcMotor motorBackLeft;
//     private DcMotor motorBackRight;
// 
//     @Override
//     public void runOpMode() {
//         // Initialize hardware
//         odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
//         motorFrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft");
//         motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");
//         motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
//         motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");
// 
//         // Set motor directions
//         motorBackRight.setDirection(DcMotor.Direction.REVERSE);
//         motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
// 
//         // Set up the odometry driver
//         odo.setOffsets(-84.0, -168.0);
//         odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//         odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
//         odo.resetPosAndIMU();
// 
//         telemetry.addData("Status", "Initialized");
//         telemetry.update();
// 
//         waitForStart();
// 
//         // Drive until y = 200
//         while (opModeIsActive()) {
//             odo.update();
//             Pose2D pos = odo.getPosition();
// 
//             // Move forward until y = 200
//             if (pos.getY(DistanceUnit.MM) < 200) {
//                 setMotorPowers(0.5, 0.5, 0.5, 0.5); // Move forward
//             } else {
//                 setMotorPowers(0, 0, 0, 0); // Stop
//                 break; // Exit the loop when y = 200
//             }
//         }
// 
//         // Translate right until x = 200
//         while (opModeIsActive()) {
//             odo.update();
//             Pose2D pos = odo.getPosition();
// 
//             // Move right until x = 200
//             if (pos.getX(DistanceUnit.MM) < 200) {
//                 translateRight(0.5); // Translate right
//             } else {
//                 setMotorPowers(0, 0, 0, 0); // Stop
//                 break; // Exit the loop when x = 200
//             }
//         }
// 
//         // Final telemetry display
//         while (opModeIsActive()) {
//             Pose2D pos = odo.getPosition();
//             String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", 
//                 pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
//             telemetry.addData("Position", data);
//             telemetry.update();
//         }
//     }
// 
//     // Set motor powers
//     private void setMotorPowers(double frontLeft, double frontRight, double backLeft, double backRight) {
//         motorFrontLeft.setPower(frontLeft);
//         motorFrontRight.setPower(frontRight);
//         motorBackLeft.setPower(backLeft);
//         motorBackRight.setPower(backRight);
//     }
// 
//     // Translate right
//     private void translateRight(double power) {
//         motorFrontLeft.setPower(power);
//         motorFrontRight.setPower(-power);
//         motorBackLeft.setPower(power);
//         motorBackRight.setPower(-power);
//     }
// }
// 