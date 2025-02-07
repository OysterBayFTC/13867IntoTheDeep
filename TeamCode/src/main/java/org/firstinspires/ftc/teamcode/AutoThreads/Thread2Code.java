package org.firstinspires.ftc.teamcode.AutoThreads;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Base.AutoRobotStruct;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Thread2Code extends LinearOpMode {

    private AutoRobotStruct robot;
    private Telemetry telemetry;
    private GoBildaPinpointDriver pinpointDriver;
    private volatile boolean movementComplete2 = false;

    // Constants for lift and movement targets
    private static final double LiftHeightoDown = 100; // height for the lift
    private static final double MIddleBlockx  = 415;//390
    private static final double MiddleBlockY  = -485;

    // Constants from AutonomousLiftTranslate
    private static final double clawOpen = 0.65;
    private static final double clawClose = 0.45;
    private static final double clawServoDrop = -1; // score position X
    private static final double clawServoPickup = 1; // score position X
    // these are the values for the rotation for the claw servo
    double clawRotateBlockDrop =.99;

    public Thread2Code(HardwareMap hardwareMap, Telemetry telemetry) {
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
    }

    @Override
    public void runOpMode() {
        // Initialize encoders for arm motors
        robot.liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        // Create and start the threads
        Thread limitThreadDown2 = new Thread(new LimitDown2());
        Thread forwardThread2 = new Thread(new ForwardMovement2());
        sleep(350);
        limitThreadDown2.start();
        forwardThread2.start();

        try {
            limitThreadDown2.join();
            forwardThread2.join();
        } catch (InterruptedException e) {
            telemetry.addData("Error", "Thread interrupted: " + e.getMessage());
            telemetry.update();
        }
    }

    private class LimitDown2 implements Runnable {
        @Override
        public void run() {
            robot.liftLeft.setPower(0.4);
            robot.liftRight.setPower(0.4);

            while (opModeIsActive() && (Math.abs(robot.liftLeft.getCurrentPosition()) > LiftHeightoDown)) {
                telemetry.addData("lift left Encoder", robot.liftLeft.getCurrentPosition());
                telemetry.addData("lift right Encoder", robot.liftRight.getCurrentPosition());
                telemetry.addData("Target Height", LiftHeightoDown);
                telemetry.update();
            }
            robot.liftLeft.setPower(0.0);
            robot.liftRight.setPower(0.0);
        }
    }

    private class ForwardMovement2 implements Runnable {
        @Override
        public void run() {
            while (!movementComplete2 && opModeIsActive()) {
                pinpointDriver.update();
                Pose2D pose = pinpointDriver.getPosition();
                double x = pose.getX(DistanceUnit.MM);
                double y =  -1 * pose.getY(DistanceUnit.MM);

                telemetry.addData("Forwards Movement X (mm)", x);
                telemetry.addData("Forwards Movement Y (mm)", y);
                telemetry.addData("Thread Status", "Moving forward");
                telemetry.update();

                if (x < MIddleBlockx ) {
                    robot.setDriverMotorPower(0.4, 0.4, 0.4, 0.4);
                } else if (y < MiddleBlockY ) {
                    robot.setDriverMotorPower(-0.5, 0.5, 0.5, -0.5);
                    robot.clawServo.setPower(clawServoPickup); // Assuming this opens the claw
                } else {
                    robot.setDriverMotorPower(0, 0, 0, 0);
                    movementComplete2 = true;
                    sleep(300);
                    robot.clawServo.setPower(clawOpen);

                    // arm going down
                    while (opModeIsActive() &&!robot.touchgrab.isPressed()) {
                        robot.ArmOne.setPower(-.4);
                        robot.ArmTwo.setPower(-.4);
                    }

                    robot.ArmOne.setPower(0);
                    robot.ArmTwo.setPower(0);
                    sleep(500);
                    robot.clawServo.setPower(clawClose); // Assuming this closes the claw
                    sleep(500);

                    while (opModeIsActive() &&!robot.touchdrop.isPressed()) {
                        robot.ArmOne.setPower(.35);
                        robot.ArmTwo.setPower(.35);
                        robot.clawRotate.setPosition(clawRotateBlockDrop);
                    }
                    robot.ArmOne.setPower(0);
                    robot.ArmTwo.setPower(0);

                    robot.clawServo.setPower(clawServoDrop); // Assuming this opens the claw
                    sleep(1000);
                    robot.ArmOne.setPower(-.3);
                    robot.ArmTwo.setPower(-.3);
                    sleep(300);
                    robot.ArmOne.setPower(0);
                    robot.ArmTwo.setPower(0);
                }
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