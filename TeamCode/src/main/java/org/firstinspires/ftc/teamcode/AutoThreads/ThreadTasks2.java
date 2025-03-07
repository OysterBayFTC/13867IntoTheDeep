package org.firstinspires.ftc.teamcode.AutoThreads;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Base.AutoRobotStruct;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class ThreadTasks2 {

    // Constants for LimitDown2 and ForwardMovement2 (if needed and not already in ThreadsTogether)
    private static final double LiftHeightoDown = 500; // height for the lift
    private static final double MIddleBlockx = 420;//390
    private static final double MiddleBlockY = 445;  // Change back to 455
    private static final double MiddleBlockY2 = 495; //505
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


    public static class LimitDown2 implements Runnable {
        private AutoRobotStruct robot;
        private Telemetry telemetry;
        private LinearOpMode opMode; // Add LinearOpMode instance

        public LimitDown2(AutoRobotStruct robot, Telemetry telemetry, LinearOpMode opMode) { // Add opMode to constructor
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
            robot.clawRotate.setPosition(clawRotateBlockLeft); // When block is left of claw


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

    public static class ForwardMovement2 implements Runnable {
        private AutoRobotStruct robot;
        private GoBildaPinpointDriver pinpointDriver;
        private Telemetry telemetry;
        private LinearOpMode opMode; // Add LinearOpMode instance
        private volatile boolean movementComplete2a = false;
        private volatile boolean movementComplete2b = false;


        public ForwardMovement2(AutoRobotStruct robot, GoBildaPinpointDriver pinpointDriver, Telemetry telemetry, LinearOpMode opMode) { // Add opMode to constructor
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

            while (!movementComplete2a && isOpModeActive()) {
                pinpointDriver.update();
                Pose2D pose = pinpointDriver.getPosition();
                double x = pose.getX(DistanceUnit.MM);
                double y =  pose.getY(DistanceUnit.MM);

                telemetry.addData("Forwards Movement X (mm)", x);
                telemetry.addData("Forwards Movement Y (mm)", y);
                telemetry.addData("Thread Status", "Moving forward");
                telemetry.update();
                if (y > MiddleBlockY) {
                    telemetry.addData("Translating", "Right");
                    telemetry.update();
                    robot.clawRotate.setPosition(clawRotateBlockLeft); // When block is left of claw
                    robot.setDriverMotorPower(-0.5, 0.5, 0.5, -0.5);
                } else if (x < MIddleBlockx) {
                    robot.setDriverMotorPower(0.4, 0.4, 0.4, 0.4);
                } else {
                    robot.clawRotate.setPosition(clawRotateBlockLeft); // When block is left of claw
                    robot.setDriverMotorPower(0, 0, 0, 0);
                    movementComplete2a = true;
                    sleep(150);

                }
            }

            while (!movementComplete2b && isOpModeActive()) {
                pinpointDriver.update();
                Pose2D pose = pinpointDriver.getPosition();
                double x = pose.getX(DistanceUnit.MM);
                double y = pose.getY(DistanceUnit.MM);

                telemetry.addData("Forwards Movement X (mm)", x);
                telemetry.addData("Forwards Movement Y (mm)", y);
                telemetry.addData("Thread Status", "Moving forward");
                telemetry.update();
                robot.clawServo.setPower(1); // Assuming this opens the claw
                if (y < MiddleBlockY2 ) {
                    // || !robot.sampleSwitch.getState()
                    telemetry.addData("Translating", "Left");
                    telemetry.update();
                    // Move in the Y direction only if the sampleSwitch is not pressed
                   robot.setDriverMotorPower(0.35, -0.35, -0.35, 0.35);


                } else {
                    // Stop movement if the sampleSwitch is pressed or Y target is reached
                    robot.setDriverMotorPower(0, 0, 0, 0);
                    robot.clawServo.setPower(0);
                    robot.clawRotate.setPosition(clawRotateBlockDrop);
                    telemetry.addData("Status", "movement 2 complete");
                    telemetry.update();
                    movementComplete2b = true;
                    sleep(150);
                }
            }
            robot.clawRotate.setPosition(clawRotateBlockDrop);
            while (isOpModeActive() && !robot.touchdrop.isPressed()) {
                robot.ArmOne.setPower(.65);
                robot.ArmTwo.setPower(.65);
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