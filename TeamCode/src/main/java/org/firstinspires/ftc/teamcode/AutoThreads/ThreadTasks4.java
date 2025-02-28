package org.firstinspires.ftc.teamcode.AutoThreads;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Base.AutoRobotStruct;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class ThreadTasks4 {

    // Constants for LimitDown2 and ForwardMovement2 (if needed and not already in ThreadsTogether)
    private static final double LiftHeightoDown = 100; // height for the lift
    private static final double RightBlockx = 495;//390
    private static final double RightBlocky = 300;
    private static final double neutralBucket = 0.35; // Catch position

    private static final double clawOpen = 0.65;
    private static final double clawClose = 0.45;
    private static final double clawServoDrop = -1; // score position X
    private static final double clawServoPickup = 1; // score position X
    private static final double dropBucket = 0.00;
    static double clawRotateBlockLeft = 0;
    static double clawRotateBlockRight = .65;
    double clawRotateBlockVert = .35;
    static double clawRotateBlockDrop = .99;

    //static double clawRotateBlockDrop = .99;

    public static class LimitDown4 implements Runnable {
        private AutoRobotStruct robot;
        private Telemetry telemetry;
        private LinearOpMode opMode; // Add LinearOpMode instance

        public LimitDown4(AutoRobotStruct robot, Telemetry telemetry, LinearOpMode opMode) { // Add opMode to constructor
            this.robot = robot;
            this.telemetry = telemetry;
            this.opMode = opMode; // Store the LinearOpMode
        }

        @Override
        public void run() {
            if (telemetry == null) {
                return; // Exit if telemetry is null
            }
            robot.bucketServo.setPosition(neutralBucket);
            robot.liftLeft.setPower(1);
            robot.liftRight.setPower(1);

            while (isOpModeActive() && (Math.abs(robot.liftLeft.getCurrentPosition()) > LiftHeightoDown)) {
                telemetry.addData("lift left Encoder", robot.liftLeft.getCurrentPosition());
                telemetry.addData("lift right Encoder", robot.liftRight.getCurrentPosition());
                telemetry.addData("Target Height", LiftHeightoDown);
                telemetry.update();
            }
            robot.liftLeft.setPower(0.0);
            robot.liftRight.setPower(0.0);
            robot.bucketServo.setPosition(dropBucket);
            sleep(1000);
            robot.bucketServo.setPosition(neutralBucket);


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

    public static class ForwardMovement4 implements Runnable {
        private AutoRobotStruct robot;
        private GoBildaPinpointDriver pinpointDriver;
        private Telemetry telemetry;
        private LinearOpMode opMode; // Add LinearOpMode instance
        private volatile boolean movementComplete4a = false;

        public ForwardMovement4(AutoRobotStruct robot, GoBildaPinpointDriver pinpointDriver, Telemetry telemetry, LinearOpMode opMode) { // Add opMode to constructor
            this.robot = robot;
            this.pinpointDriver = pinpointDriver;
            this.telemetry = telemetry;
            this.opMode = opMode; // Store the LinearOpMode
        }

        @Override
        public void run() {
            if (telemetry == null) {
                return; // Exit if telemetry is null
            }

            double desiredHeading = 0; // Desired heading for straight path
            double kP = 0.05; // Proportional gain for heading correction

            while (!movementComplete4a && isOpModeActive()) {
                pinpointDriver.update();
                Pose2D pose = pinpointDriver.getPosition();
                double heading = pinpointDriver.getHeading(AngleUnit.DEGREES);
                double x = pose.getX(DistanceUnit.MM);
                double y = pose.getY(DistanceUnit.MM);

                telemetry.addData("Forwards Movement X (mm)", x);
                telemetry.addData("Forwards Movement Y (mm)", y);
                telemetry.addData("Forwards Movement Heading (deg)", heading);
                telemetry.addData("Thread Status", "Moving forward");
                telemetry.update();

                // Calculate heading error
                double headingError = desiredHeading - heading;

                // Adjust motor powers based on heading error
                double leftPower = 0.4 + kP * headingError;
                double rightPower = 0.4 - kP * headingError;

                robot.clawServo.setPower(1); // Assuming this opens the claw
                if (x < RightBlockx) {
                    robot.setDriverMotorPower(leftPower, rightPower, leftPower, rightPower);
                    robot.clawRotate.setPosition(clawRotateBlockRight);
                } else if (y > RightBlocky) {
                    robot.setDriverMotorPower(-0.25, 0.25, 0.25, -0.25);
                } else {
                    robot.setDriverMotorPower(0, 0, 0, 0);
                    movementComplete4a = true;
                    sleep(150);
                }


            }
            robot.clawRotate.setPosition(clawRotateBlockDrop);
            while (isOpModeActive() && !robot.touchdrop.isPressed()) {
                robot.ArmOne.setPower(.4);
                robot.ArmTwo.setPower(.4);
            }
            robot.ArmOne.setPower(0);
            robot.ArmTwo.setPower(0);
            robot.clawServo.setPower(-1.0);
            sleep(1000);

            robot.ArmOne.setPower(-.7);
            robot.ArmTwo.setPower(-.7);
            sleep(400);
            robot.ArmOne.setPower(0);
            robot.ArmTwo.setPower(0);
            robot.clawServo.setPower(0);
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
