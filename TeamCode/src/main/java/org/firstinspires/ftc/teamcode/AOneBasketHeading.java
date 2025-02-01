package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Base.AutoRobotStruct;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AOneBasketHeading", group = "Autonomous")
public class AOneBasketHeading extends LinearOpMode {

    private AutoRobotStruct robot = new AutoRobotStruct(); // Hardware structure
    private GoBildaPinpointDriver pinpointDriver;
    private volatile boolean movementComplete1 = false;
    private static final double TARGET_POSITIONX1 = 100;
    private static final double TARGET_POSITIONY1 = 540.0;
    private static final double targeLifttHeight = 5600; // height for the lift
    private static final double targeLifttHeightdown = 100; // height for the lift
    private static final double dropBucket = 0.00;
    private static final double neutralBucket = 0.35; // Catch position
    private static final double startBucket = 0.45; // Catch position
    private static final double clawOpen = 0.65;
    private static final double clawClose = 0.45;
    // these are the values for the rotation for the claw servo
    double clawRotateBlockLeft = 0;
    double clawRotateBlockRight = .65;
    double clawRotateBlockVert = .35;
    double clawRotateBlockDrop = .99;
    private final double clawStepUp = 0.20;
    private final double clawStepDown = .20;
    private static final double clawServoDrop = 0; // score position X
    private static final double clawServoPickup = 0.5; // score position X

    @Override
    public void runOpMode() {
        // Initialize robot hardware
        robot.initHardware(hardwareMap);
        robot.bucketServo.setPosition(startBucket); // Set bucketServo to 0.35 initially
        robot.clawRotate.setPosition(clawRotateBlockDrop);

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
        robot.ArmOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.ArmTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        SharedState.liftEncoderValue = robot.liftLeft.getCurrentPosition(); // Or use liftRight if needed
        SharedState.liftEncoderValue = robot.liftRight.getCurrentPosition(); // Or use liftRight if needed
        // Start the first set of movements
        Thread liftThread1 = new Thread(new LiftTask1());
        Thread movementThread1 = new Thread(new MovementTask1());
        sleep(350);
        liftThread1.start();
        movementThread1.start();

        try {
            liftThread1.join();
            movementThread1.join();
        } catch (InterruptedException e) {
            telemetry.addData("Error", "Thread interrupted: " + e.getMessage());
            telemetry.update();
        }
    }

    // Task to handle bringing the lift up
    private class LiftTask1 implements Runnable {
        @Override
        public void run() {
            robot.ArmOne.setPower(-0.65);  // Optimized power from version 2
            robot.ArmTwo.setPower(-0.65);
            sleep(400);  // Optimized sleep from version 2
            robot.ArmOne.setPower(-0.2);
            robot.ArmTwo.setPower(0.2);
            robot.bucketServo.setPosition(neutralBucket);
            robot.clawRotate.setPosition(clawRotateBlockRight);

            robot.liftLeft.setPower(-0.4);
            robot.liftRight.setPower(-0.4);

            while (opModeIsActive() && (Math.abs(robot.liftLeft.getCurrentPosition()) < targeLifttHeight)) {
                // telemetry.addData("lift left Encoder", robot.liftLeft.getCurrentPosition());
                // telemetry.addData("lift right Encoder", robot.liftRight.getCurrentPosition());
                // telemetry.addData("Target Height", targeLifttHeight);
                // telemetry.update();
            }

            robot.liftLeft.setPower(0.05);
            robot.liftRight.setPower(0.05);
            sleep(300);

            robot.bucketServo.setPosition(dropBucket);
            sleep(1000);
        }
    }

    // Task to handle movement based on odometry feedback and gyro correction
    private class MovementTask1 implements Runnable {
        @Override
        public void run() {
            double targetHeading = pinpointDriver.getHeading(); // Get the initial heading
            double Kp = 0.02; // Proportional gain for gyro correction (adjust as needed)

            while (!movementComplete1 && opModeIsActive()) {
                pinpointDriver.update();
                Pose2D pose = pinpointDriver.getPosition();
                double x = pose.getX(DistanceUnit.MM);
                double y = pose.getY(DistanceUnit.MM);
                double currentHeading = pinpointDriver.getHeading(); // Get current heading from the gyro

                // Calculate the error in heading (difference between target and current heading)
                double headingError = targetHeading - currentHeading;

                // Adjust motor powers based on heading error to maintain a straight path
                double correction = headingError * Kp;

                if (x < TARGET_POSITIONX1) {
                    // Move forward while applying gyro correction
                    robot.setDriverMotorPower(0.4 - correction, 0.4 + correction, 0.4 - correction, 0.4 + correction);
                } else if (y < TARGET_POSITIONY1) {
                    // Strafe left while applying gyro correction
                    robot.setDriverMotorPower(0.5 - correction, -0.5 + correction, -0.5 - correction, 0.5 + correction);
                } else {
                    robot.setDriverMotorPower(0, 0, 0, 0); // Stop motors
                    movementComplete1 = true;
                }

                telemetry.addData("X (mm)", x);
                telemetry.addData("Y (mm)", y);
                telemetry.addData("Target Heading", targetHeading);
                telemetry.addData("Current Heading", currentHeading);
                telemetry.addData("Heading Error", headingError);
                telemetry.addData("Correction", correction);
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