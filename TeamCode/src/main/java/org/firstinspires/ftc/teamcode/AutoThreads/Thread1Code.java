package org.firstinspires.ftc.teamcode.AutoThreads;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Base.AutoRobotStruct;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Thread1Code extends LinearOpMode {

    private AutoRobotStruct robot;
    private Telemetry telemetry;
    private GoBildaPinpointDriver pinpointDriver;
    private volatile boolean movementComplete1 = false;

    // Constants for lift and movement targets
    private static final double LiftHeightUp  = 5600; // height for the lift
    private static final double InitalTargetx = 100;
    private static final double InitalTargety = 540.0;

    // Constants from AutonomousLiftTranslate
    private static final double dropBucket = 0.00;
    private static final double neutralBucket = 0.35; // Catch position
    private static final double startBucket = 0.45; // Catch position
    // these are the values for the rotation for the claw servo
    double clawRotateBlockLeft = 0;
    double clawRotateBlockRight =.65;
    double clawRotateBlockVert =.35;
    double clawRotateBlockDrop =.99;

    // Constructor to receive hardwareMap and telemetry
    public Thread1Code(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap; // Initialize hardwareMap
        robot = new AutoRobotStruct();
        robot.initHardware(hardwareMap);
        pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // Set up Pinpoint driver configuration
        pinpointDriver.setOffsets(-84.0, -168.0);
        pinpointDriver.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpointDriver.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpointDriver.resetPosAndIMU();

        // Initialize robot hardware (from AutonomousLiftTranslate)
        robot.bucketServo.setPosition(startBucket); // Set bucketServo to 0.35 initially
        robot.clawRotate.setPosition(clawRotateBlockDrop);
    }

    @Override
    public void runOpMode() {
        // Initialize encoders for arm motors
        robot.liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize encoders for arm motors (from AutonomousLiftTranslate)
        robot.ArmOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.ArmTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        // Start the threads
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

    // Lift task (from LiftTask1)
    private class LiftTask1 implements Runnable {
        @Override
        public void run() {
            robot.ArmOne.setPower(-0.85);
            robot.ArmTwo.setPower(-0.85);
            sleep(400);
            robot.ArmOne.setPower(-0.2);
            robot.ArmTwo.setPower(0.2);
            robot.bucketServo.setPosition(neutralBucket);
            robot.clawRotate.setPosition(clawRotateBlockRight);

            robot.liftLeft.setPower(-0.4);
            robot.liftRight.setPower(-0.4);

            while (opModeIsActive() && (Math.abs(robot.liftLeft.getCurrentPosition()) < LiftHeightUp)) {
                // Telemetry and loop condition as needed
            }

            robot.liftLeft.setPower(0.05);
            robot.liftRight.setPower(0.05);
            sleep(300);

            robot.bucketServo.setPosition(dropBucket);
            sleep(1000);
            //... rest of the lift logic
        }
    }

    // Movement task (from MovementTask1)
    private class MovementTask1 implements Runnable {
        @Override
        public void run() {
            while (!movementComplete1 && opModeIsActive()) {
                pinpointDriver.update();
                Pose2D pose = pinpointDriver.getPosition();
                double x = pose.getX(DistanceUnit.MM);
                double y = pose.getY(DistanceUnit.MM);

                if (x < InitalTargetx) {
                    robot.setDriverMotorPower(0.4, 0.4, 0.4, 0.4);
                } else if (y < InitalTargety) {
                    robot.setDriverMotorPower(0.5, -0.5, -0.5, 0.5);
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