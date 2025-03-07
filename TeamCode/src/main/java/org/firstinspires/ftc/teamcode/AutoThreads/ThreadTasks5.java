package org.firstinspires.ftc.teamcode.AutoThreads;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Base.AutoRobotStruct;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class ThreadTasks5 {
    // Constants for LiftTask1 and ForwardMovement3 (if needed and not already in ThreadsTogether)
    private static final double LiftHeightUp = 5600; // height for the lift
    private static final double ScorePositionX = 370;
    private static final double ScorePositionY = 520.0;
    private static final double neutralBucket = 0.35; // Catch position
    private static final double dropBucket = 0.00;
    static double clawRotateBlockRight = .65;
    static double clawRotateBlockLeft = .0;
    static double clawRotateBlockDrop = .99;
    private static final double clawServoPickup = 1; // score position X

    //Lift Task 5 - Runnable Class
    public static class LiftTask5 implements Runnable {
        private AutoRobotStruct robot;
        private Telemetry telemetry;
        private LinearOpMode opMode;
        public LiftTask5(AutoRobotStruct robot, Telemetry telemetry, LinearOpMode opMode) {
            this.robot = robot;
            this.telemetry = telemetry;
            this.opMode = opMode;
        }

        @Override
        public void run() {
            if (telemetry == null) {
                telemetry.addData("Error", "Telemetry is null");
                telemetry.update();
                return; // Exit if telemetry is null
            }
            robot.bucketServo.setPosition(neutralBucket);
            robot.liftLeft.setPower(-1);
            robot.liftRight.setPower(-1);
            while (isOpModeActive() && (Math.abs(robot.liftLeft.getCurrentPosition()) < LiftHeightUp)) {
                telemetry.addData("Lift Position", robot.liftLeft.getCurrentPosition());
                telemetry.update();
            }

            robot.liftLeft.setPower(0.05);
            robot.liftRight.setPower(0.05);
            sleep(300);
            robot.bucketServo.setPosition(dropBucket);
            sleep(1000);

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
    // Movement Task 5 (copied from ForwardMovement3) - Runnable Class
    public static class ForwardMovement5 implements Runnable {
        private AutoRobotStruct robot;
        private GoBildaPinpointDriver pinpointDriver;
        private Telemetry telemetry;
        private LinearOpMode opMode;
        private volatile boolean movementComplete5 = false;

        public ForwardMovement5(AutoRobotStruct robot, GoBildaPinpointDriver pinpointDriver, Telemetry telemetry, LinearOpMode opMode){

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
            double desiredHeading = 0; // Desired heading for straight path
            double kP = 0.05; // Proportional gain for heading correction

            while (!movementComplete5 && isOpModeActive()) {
                pinpointDriver.update();
                Pose2D pose = pinpointDriver.getPosition();
                double heading = pinpointDriver.getHeading(AngleUnit.DEGREES);
                double x = pose.getX(DistanceUnit.MM);
                double y = pose.getY(DistanceUnit.MM);




                telemetry.addData("X (mm)", x);
                telemetry.addData("Y (mm)", y);
                telemetry.addData("Heading (deg)", heading);
                telemetry.update();

                // Move in x direction
                if (x > ScorePositionX) {
                    robot.setDriverMotorPower(-0.45, -0.45, -0.45, -0.45);
            }
                else if (y < ScorePositionY){
                    robot.setDriverMotorPower(0.30, -0.30, -0.30, 0.30);
                }
                else{
                    robot.setDriverMotorPower(0, 0, 0, 0);
                    movementComplete5 = true;
                }
        }
            if (movementComplete5) {
                while (isOpModeActive() && !robot.touchgrab.isPressed()) {
                    robot.ArmOne.setPower(-.5);
                    robot.ArmTwo.setPower(-.5);
                }
                robot.ArmOne.setPower(0);
                robot.ArmTwo.setPower(0);
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
