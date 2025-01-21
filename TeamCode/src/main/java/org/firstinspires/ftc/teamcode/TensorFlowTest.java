// package org.firstinspires.ftc.teamcode;
// 
// import org.firstinspires.ftc.teamcode.Base.AutoRobotStruct;
// import com.qualcomm.robotcore.hardware.GyroSensor;
// import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// import com.qualcomm.robotcore.util.ElapsedTime;
// 
// @Autonomous(name = "TensorFlowTest")
// public class TensorFlowTest extends AutoRobotStruct {
// 
//     private GyroSensor gyroSensor;
//     private ElapsedTime runtime = new ElapsedTime();
// 
//     @Override
//     public void runOpMode() {
//         initRunner();
//         initGyro();
// 
//         waitForStart();
//         while (opModeIsActive()) {
//             
//             // Perform a 90-degree right turn
//             
//             turnRight(88.5);
//             sleep(300000);
//             // Perform a 90-degree left turn
//         }
//     }
// 
//     private void initGyro() {
//         gyroSensor = hardwareMap.get(GyroSensor.class, "gyroSensor");
//         gyroSensor.calibrate();
// 
//         while (gyroSensor.isCalibrating()) {
//             telemetry.addData("Status", "Calibrating gyro...");
//             telemetry.update();
//             sleep(50);
//         }
// 
//         telemetry.addData("Status", "Gyro Calibration Complete");
//         telemetry.update();
//     }
// 
//    private void turnLeft(double targetAngle) {
//         double initialAngle = gyroSensor.getHeading();
//         double currentAngle = initialAngle;
// 
//         while (opModeIsActive() && Math.abs(currentAngle - initialAngle) < targetAngle) {
//             setDriverMotorPower(0.15, -0.15, 0.15, -0.15);
//             telemetry.addData("Turning Left", "Current Angle: %.2f", currentAngle);
//             telemetry.update();
// 
//             currentAngle = gyroSensor.getHeading();
//         }
// 
//         setDriverMotorZero();
//     }   
// 
//     private void turnRight(double targetAngle) {
//      double initialAngle = gyroSensor.getHeading();
//         double currentAngle = initialAngle;
// 
//         while (opModeIsActive() && Math.abs(currentAngle - initialAngle) < targetAngle) {
//             setDriverMotorPower(-0.15, 0.15, -0.15, 0.15);
//             telemetry.addData("Turning Right", "Current Angle: %.2f", currentAngle);
//             telemetry.update();
// 
//             currentAngle = gyroSensor.getHeading();
//         }
// 
//         setDriverMotorZero();
//         return;
//     }   
// }
// 
// 