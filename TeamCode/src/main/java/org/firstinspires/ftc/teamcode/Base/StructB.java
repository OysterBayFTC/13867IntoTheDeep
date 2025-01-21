// package org.firstinspires.ftc.teamcode.base;
// import com.qualcomm.hardware.bosch.BNO055IMU;
// import com.qualcomm.robotcore.hardware.TouchSensor;
// import com.qualcomm.robotcore.eventloop.opmode.OpMode;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.DcMotorEx;
// import com.qualcomm.robotcore.hardware.Servo;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
// import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
// import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
// import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
// import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
// import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
// import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
// import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
// 
// import java.util.Locale;
// 
// import static java.lang.Double.parseDouble;
// 
// public class StructB extends OpMode {
//     DcMotor motorFrontRight;
//     DcMotor motorFrontLeft;
//     DcMotor motorBackRight;
//     DcMotor motorBackLeft;
//    
// 
// 
//     @Override
//     public void init() {
//         
//         motorFrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft"); // 3
//         motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight"); // 2
//         motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft"); // 1
//         motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight"); // 0
// 
// 
//         
// 
//         telemetry.addData("Status", "Initialized");
//         telemetry.update();
//     
//         motorBackRight.setDirection(DcMotor.Direction.REVERSE);
//         motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
//    
// 
// 
//     }
// 
//     @Override
//     public void loop() {}
//     public void initDriver(){
//         float gamepad1LeftY = gamepad1.left_stick_y;
//         float gamepad1LeftX = gamepad1.left_stick_x;
//         float gamepad2RightY = gamepad2.right_stick_y;
//         float gamepad1RightX = -gamepad1.right_stick_x;
//         float frontRightPower = -gamepad1LeftY + gamepad1LeftX + gamepad1RightX;
//         float frontLeftPower = -gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
//         float backLeftPower = -gamepad1LeftY + gamepad1LeftX - gamepad1RightX;
//         float backRightPower = -gamepad1LeftY - gamepad1LeftX + gamepad1RightX;
// 
// 
//         motorFrontLeft.setPower(frontLeftPower);
//         motorBackLeft.setPower(backLeftPower);
//         motorFrontRight.setPower(frontRightPower);
//         motorBackRight.setPower(backRightPower);
//         
// 
//             
//             
//     }  public void setDriverMotorPower(double FRightPower, double FLeftPower, double BRightPower, double BLeftPower) {
//         motorFrontRight.setPower(FRightPower);
//         motorFrontLeft.setPower(FLeftPower);
//         motorBackLeft.setPower(BLeftPower);
//         motorBackRight.setPower(BRightPower);
//     }
// 
//     public void setDriverPowerZERO() {
//         motorFrontRight.setPower(0);
//         motorFrontLeft.setPower(0);
//         motorBackLeft.setPower(0);
//         motorBackRight.setPower(0);
//     }
// 
//     public void translateRight(double m) {
//         motorFrontRight.setPower(-m);
//         motorFrontLeft.setPower(m);
//         motorBackLeft.setPower(-m);
//         motorBackRight.setPower(m);
//     }
// 
//     public void translateLeft(double m) {
//         motorFrontRight.setPower(m);
//         motorFrontLeft.setPower(-m);
//         motorBackLeft.setPower(m);
//         motorBackRight.setPower(-m);
//         
//     } 
//     
// }