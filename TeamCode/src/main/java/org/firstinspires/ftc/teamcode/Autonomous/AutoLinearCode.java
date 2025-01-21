package org.firstinspires.ftc.teamcode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Base.AutoRobotStruct;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;


@Autonomous(name = "AutoLinearCode", group = "Autonomous")
public class AutoLinearCode extends LinearOpMode {
     private volatile boolean movementComplete1 = false;
   // private volatile boolean movementComplete2 = false;
   // private volatile boolean movementComplete3 = false;
    private AutoRobotStruct robot = new AutoRobotStruct(); // Hardware structure
    private GoBildaPinpointDriver pinpointDriver;
    private static final double TARGET_POSITIONX1 = 125.0;
    private static final double TARGET_POSITIONY1 = -550.0;
    private static final double middle_block_pickupX2 = 405;
    private static final double middle_block_pickup_Y2 = -549;
     private static final double right_block_pickupX4 = 406;
    private static final double right_block_pickup_Y4 = -509;
    private static final double targeLifttHeight = 5400; // height for the lift 
    private static final double targeLifttHeightdown = 900; // height for the lift  
    private static final double scorePositionX1 = 200; // score position X  
    private static final double scorePositionY1 = -550; // score position Y  
    private static final double dropBucket = 0.02;
    private static final double neutralBucket = 0.35; // Catch position

    @Override
    public void runOpMode() {
        // Initialize robot hardware
        robot.initHardware(hardwareMap);
        robot.bucketServo.setPosition(neutralBucket); // Set bucketServo to 0.35 initially

        // Initialize GoBilda Pinpoint
        pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // Set up Pinpoint driver configuration
        pinpointDriver.setOffsets(-84.0, -168.0);
        pinpointDriver.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpointDriver.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpointDriver.resetPosAndIMU();

        // Initialize encoders for arm motors
        robot.liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); 
        robot.liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); 
        robot.liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER); 
        robot.liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER); 

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        
          while (!movementComplete1 && opModeIsActive()) {
                pinpointDriver.update(); 
                Pose2D pose = pinpointDriver.getPosition();
                double x = pose.getX(DistanceUnit.MM);
                double y = pose.getY(DistanceUnit.MM);

                if (x < TARGET_POSITIONX1) {
                    robot.setDriverMotorPower(0.4, 0.4, 0.4, 0.4); 
                } else {
                    robot.setDriverMotorPower(0, 0, 0, 0); 
                    movementComplete1 = true;
                }

                telemetry.addData("X (mm)", x);
                telemetry.addData("Y (mm)", y);
                telemetry.update();
            }
      
    }
}