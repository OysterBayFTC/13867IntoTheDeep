package org.firstinspires.ftc.teamcode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Base.AutoRobotStruct;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name = "Autonomous_test", group = "Autonomous")
public class Autonomous_test extends LinearOpMode {

    private AutoRobotStruct robot = new AutoRobotStruct(); // Hardware structure
    private GoBildaPinpointDriver pinpointDriver;
    private volatile boolean movementComplete = false;
    private static final double TARGET_POSITION = 500.0;

    @Override
    public void runOpMode() {
        // Initialize robot hardware
        robot.initHardware(hardwareMap);

        // Initialize GoBilda Pinpoint
        pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // Set up Pinpoint driver configuration
        pinpointDriver.setOffsets(-84.0, -168.0);
        pinpointDriver.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpointDriver.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpointDriver.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        Thread liftThread = new Thread(new LiftTask());
        Thread movementThread = new Thread(new MovementTask());

        liftThread.start();
        movementThread.start();

        try {
            liftThread.join(); // Ensure the lift thread completes
            movementThread.join(); // Ensure the movement thread completes
        } catch (InterruptedException e) {
            telemetry.addData("Error", "Thread interrupted: " + e.getMessage());
            telemetry.update();
        }
    }

    // Task to handle the lifting operation
    private class LiftTask implements Runnable {
        @Override
        public void run() {
            robot.ArmOne.setPower(0.45);
            robot.ArmTwo.setPower(-0.45);
           sleep(2000); 
            robot.ArmOne.setPower(0);
            robot.ArmTwo.setPower(0);
          
            robot.liftLeft.setPower(0.75);
            robot.liftRight.setPower(-0.75);
            sleep(2000);
            robot.liftLeft.setPower(0.1);
            robot.liftRight.setPower(-0.1);
    
        }
    }

    // Task to handle movement based on odometry feedback
private class MovementTask implements Runnable {
    @Override
    public void run() {
        while (!movementComplete && opModeIsActive()) {
            pinpointDriver.update(); // Refresh odometry data
            Pose2D pose = pinpointDriver.getPosition();
            double x = pose.getX(DistanceUnit.MM);
            double y = pose.getY(DistanceUnit.MM);

            // Move forward until y reaches TARGET_POSITION
            // Strafe right until x reaches TARGET_POSITION
            if (x < TARGET_POSITION) {
                robot.setDriverMotorPower(0.4, 0.4, 0.4, 0.4); // Move forward
            } 
            else if (y < TARGET_POSITION) {
                robot.setDriverMotorPower(-0.20, 0.2, 0.2, -0.2); // Strafe right
            } 
            // Stop when both x and y have reached or exceeded the target
            else {
   //             bucketServo.setPosition(0.5); // if this does nothing, switch to other servor position.
                robot.setDriverMotorPower(0, 0, 0, 0); // Stop all motors
                movementComplete = true;
            }

            telemetry.addData("X (mm)", x);
            telemetry.addData("Y (mm)", y);
            telemetry.update();
        }
    }
    
}

    // Helper method to sleep without exception interruptions
    private void sleep(int milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            telemetry.addData("Error", "Sleep interrupted: " + e.getMessage());
            telemetry.update();
        }
    }
}
