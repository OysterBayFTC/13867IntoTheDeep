// package org.firstinspires.ftc.teamcode;
// 
// import org.firstinspires.ftc.teamcode.Base.AutoRobotStruct;
// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
// import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// 
// @Autonomous
// 
// public class ParkingAutonomous extends LinearOpMode {
//     private AutoRobotStruct robot = new AutoRobotStruct(); // Hardware structure
//     private GoBildaPinpointDriver pinpointDriver;
// @Override
//     public void runOpMode() {
//         robot.initHardware(hardwareMap);
//         pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
//         
//         pinpointDriver.setOffsets(-84.0, -168.0);
//         pinpointDriver.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//         pinpointDriver.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
//         pinpointDriver.resetPosAndIMU();
//         
//         telemetry.addData("Status", "Initialized");
//         telemetry.update();
//         
//         waitForStart();
//         if (opModeIsActive()) {
//         setDriverMotorPower(-0.20, -0.20, -0.20, -0.20, 4200);    
//     }
//     }
// }