package org.firstinspires.ftc.teamcode.AutoThreads;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Base.AutoRobotStruct;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class ThreadTasks1 {

    // Constants for LiftTask1 and MovementTask1 (if needed and not already in ThreadsTogether)
    private static final double LiftHeightUp  = 5600; // height for the lift
    private static final double InitalTargetx = 120;
    private static final double InitalTargety = 540.0;
    private static final double neutralBucket = 0.35; // Catch position
    private static final double dropBucket = 0.00;
    static double clawRotateBlockRight =.65;
    static double clawRotateBlockLeft =.0;

    double clawRotateBlockDrop =.99;
    private static final double clawServoPickup = 1; // score position X

    // Lift task (from LiftTask1) - Runnable Class
    public static class LiftTask1 implements Runnable {
        private AutoRobotStruct robot;
        private Telemetry telemetry;
        private LinearOpMode opMode; // Add LinearOpMode instance

        public LiftTask1(AutoRobotStruct robot, Telemetry telemetry, LinearOpMode opMode) { // Add opMode to constructor
            this.robot = robot;
            this.telemetry = telemetry;
            this.opMode = opMode; // Store the LinearOpMode
        }

        @Override
        public void run() {
            if (telemetry == null) {
                return; // Exit if telemetry is null
            }

            robot.ArmOne.setPower(-0.65);
            robot.ArmTwo.setPower(-0.65);
            sleep(400);
            robot.ArmOne.setPower(0);
            robot.ArmTwo.setPower(0);

            robot.bucketServo.setPosition(neutralBucket);
            robot.liftLeft.setPower(-0.85);
            robot.liftRight.setPower(-0.85);

            while (isOpModeActive() && (Math.abs(robot.liftLeft.getCurrentPosition()) < LiftHeightUp)) {
                // Telemetry and loop condition as needed
            }

            robot.liftLeft.setPower(0.05);
            robot.liftRight.setPower(0.05);
            sleep(300);

            robot.bucketServo.setPosition(dropBucket);
            robot.bucketServo.setPosition(dropBucket);
            sleep(1000);
            //... rest of the lift logic
        }
        private void sleep(int milliseconds) {
            try {
                Thread.sleep(milliseconds);
            } catch (InterruptedException e) {
                telemetry.addData("Error", "Sleep interrupted: " + e.getMessage());
                telemetry.update();
                Thread.currentThread().interrupt();
            }
        }
        private boolean isOpModeActive() {
            return opMode.opModeIsActive();
        }
    }

    // Movement task (from MovementTask1) - Runnable Class
    public static class MovementTask1 implements Runnable {
        private AutoRobotStruct robot;
        private GoBildaPinpointDriver pinpointDriver;
        private Telemetry telemetry;
        private LinearOpMode opMode;
        private volatile boolean movementComplete1 = false;

        public MovementTask1(AutoRobotStruct robot, GoBildaPinpointDriver pinpointDriver, Telemetry telemetry, LinearOpMode opMode) {
            this.robot = robot;
            this.pinpointDriver = pinpointDriver;
            this.telemetry = telemetry;
            this.opMode = opMode;
        }

        @Override
        public void run() {
            if (telemetry == null) {
                return; // Exit if telemetry is null
            }

            while (!movementComplete1 && isOpModeActive()) {
                pinpointDriver.update();
                Pose2D pose = pinpointDriver.getPosition();
                double x = pose.getX(DistanceUnit.MM);
                double y = pose.getY(DistanceUnit.MM);

                // Move in X direction
                if (x < InitalTargetx) {
                    robot.setDriverMotorPower(0.4, 0.4, 0.4, 0.4);
                }
                // Move in Y direction
                else if (y < InitalTargety) {
                    robot.setDriverMotorPower(0.5, -0.5, -0.5, 0.5);
                }
                // Stop and set movementComplete1 to true
                else {
                    robot.setDriverMotorPower(0, 0, 0, 0);
                    movementComplete1 = true;
                }

                telemetry.addData("X (mm)", x);
                telemetry.addData("Y (mm)", y);
                telemetry.update();
            }

            // After movement is complete, interact with the arm and claw
            if (movementComplete1) {
                while (isOpModeActive() && !robot.touchgrab.isPressed()) {
                    robot.ArmOne.setPower(-.1);
                    robot.ArmTwo.setPower(-.1);
                }


                robot.ArmOne.setPower(0);
                robot.ArmTwo.setPower(0);
                robot.clawRotate.setPosition(clawRotateBlockLeft); // When block is left of claw

                telemetry.addData("Status", "Arm and claw interaction complete");
                telemetry.update();
            }
        }

        private void sleep(int milliseconds) {
            try {
                Thread.sleep(milliseconds);
            } catch (InterruptedException e) {
                telemetry.addData("Error", "Sleep interrupted: " + e.getMessage());
                telemetry.update();
                Thread.currentThread().interrupt();
            }
        }

        private boolean isOpModeActive() {
            return opMode.opModeIsActive();
        }
    }
}