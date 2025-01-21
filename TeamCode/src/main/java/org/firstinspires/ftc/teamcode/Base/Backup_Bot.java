// package org.firstinspires.ftc.teamcode.base;
// 
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.Servo;
// import com.qualcomm.robotcore.hardware.DigitalChannel;
// import com.qualcomm.robotcore.eventloop.opmode.OpMode;
// 
// public class Backup_Bot extends OpMode {
//   //  DcMotor motorOne;
//   //  Servo servoOne;
// //    DcMotor motorTwo;
//  //   Servo servoTwo;
//     DigitalChannel touch; // Rename touch sensor to "touch"
// 
//     @Override
//     public void init() {
//         motorOne = hardwareMap.get(DcMotor.class, "motorOne");
//         servoOne = hardwareMap.get(Servo.class, "servoOne");
//         motorTwo = hardwareMap.get(DcMotor.class, "motorTwo");
//         servoTwo = hardwareMap.get(Servo.class, "servoTwo");
//         
//         touch = hardwareMap.get(DigitalChannel.class, "touch"); // Use the renamed touch sensor
//         touch.setMode(DigitalChannel.Mode.INPUT);
//     }
// 
//     @Override
//     public void loop() {
//         initDriver();
//         updateTelemetry();
//     }
// 
//     public void initDriver() {
//         // Run motors when button A is pressed, until the touch sensor is pressed
//         if (gamepad1.a) {
//             motorOne.setPower(0.5);
//             motorTwo.setPower(0.5);
//         } else if (gamepad1.y) {
//             motorOne.setPower(-0.5);
//             motorTwo.setPower(-0.5);
//         } else {
//             motorOne.setPower(0);
//             motorTwo.setPower(0);
//         }
// 
//         // Stop motors if the touch sensor is pressed
//         if (!touch.getState()) { // Assuming the sensor is active low
//             motorOne.setPower(0);
//             motorTwo.setPower(0);
//         }
// 
//         if (gamepad2.a) {
//             servoTwo.setPosition(0.40);
//         }
//         if (gamepad2.b) {
//             servoTwo.setPosition(0.10);
//         }
//         
//         if (gamepad2.y) {
//             servoOne.setPosition(0.36);
//         }
//         
//         if (gamepad2.x) {
//             servoOne.setPosition(0.15);
//         }
//     }
// 
//     private void updateTelemetry() {
//         // Display the state of the touch sensor
//         boolean touchState = touch.getState(); // Get the state of the touch sensor
//         telemetry.addData("Touch Sensor", touchState ? "Released" : "Pressed"); // Display the state
//         telemetry.update(); // Update the telemetry display
//     }
// }