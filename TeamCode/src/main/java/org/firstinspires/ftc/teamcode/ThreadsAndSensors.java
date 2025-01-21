// package org.firstinspires.ftc.teamcode;
// 
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
// import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
// import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
// import java.util.Locale;
// // import org.firstinspires.ftc.teamcode.Base;
// @TeleOp(name="goBILDA® PinPoint Odometry Example", group="Linear OpMode")
// //@Disabled
// public class ThreadsAndSensors extends LinearOpMode {
// 
//     GoBildaPinpointDriver odo;
//     DcMotor motorX;
//     DcMotor motorY;
//     double oldTime = 0;
// 
//     @Override
//     public void runOpMode() {
//         odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
//         motorX = hardwareMap.get(DcMotor.class, "motorX"); // Replace with your motor name
//         motorY = hardwareMap.get(DcMotor.class, "motorY"); // Replace with your motor name
// 
//         odo.setOffsets(-84.0, -168.0);
//         odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//         odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
//         odo.resetPosAndIMU();
// 
//         telemetry.addData("Status", "Initialized");
//         telemetry.update();
// 
//         waitForStart();
//         resetRuntime();
// 
//         Thread moveXThread = new Thread(new Runnable() {
//             @Override
//             public void run() {
//                 while (opModeIsActive()) {
//                     odo.update();
//                     Pose2D pos = odo.getPosition();
//                     if (pos.getX(DistanceUnit.MM) < 75) {
//                         motorX.setPower(0.5); // Adjust power as needed
//                     } else {
//                         motorX.setPower(0);
//                     }
//                 }
//             }
//         });
// 
//         Thread moveYThread = new Thread(new Runnable() {
//             @Override
//             public void run() {
//                 while (opModeIsActive()) {
//                     odo.update();
//                     Pose2D pos = odo.getPosition();
//                     if (pos.getY(DistanceUnit.MM) < 75) {
//                         motorY.setPower(0.5); // Adjust power as needed
//                     } else {
//                         motorY.setPower(0);
//                     }
//                 }
//             }
//         });
// 
//         moveXThread.start();
//         moveYThread.start();
// 
//         while (opModeIsActive()) {
//             double newTime = getRuntime();
//             double loopTime = newTime - oldTime;
//             double frequency = 1 / loopTime;
//             oldTime = newTime;
// 
//             Pose2D pos = odo.getPosition();
//             String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
//             telemetry.addData("Position", data);
// 
//             telemetry.addData("Status", odo.getDeviceStatus());
//             telemetry.addData("Pinpoint Frequency", odo.getFrequency());
//             telemetry.addData("REV Hub Frequency: ", frequency);
//             telemetry.update();
//         }
//     }
// }
// 