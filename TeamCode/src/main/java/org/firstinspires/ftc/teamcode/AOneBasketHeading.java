package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Base.AutoRobotStruct;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AOneBasketHeading", group = "Autonomous")
public class AOneBasketHeading extends LinearOpMode {

    private AutoRobotStruct robot = new AutoRobotStruct();
    private GoBildaPinpointDriver pinpointDriver;

    private static final double TARGET_POSITIONX1 = 80;
    private static final double TARGET_POSITIONY1 = 540.0;
    private static final double TARGET_POSITIONX2 = 470; // Target X for second movement
    private static final double TARGET_POSITIONY2 = 200; // Target Y for second movement
    private static final double TARGET_POSITIONX3 = 500;
    private static final double TARGET_POSITIONY3 = 150.0;
    private static final double TARGET_LIFT_HEIGHT = 5600;
    private static final double TARGET_LIFT_HEIGHT_DOWN = 100;
    private static final double DROP_BUCKET = 0.00;
    private static final double NEUTRAL_BUCKET = 0.35;
    private static final double START_BUCKET = 0.45;
    private static final double CLAW_OPEN = 0.65;
    private static final double CLAW_CLOSE = 0.45;
    private static final double CLAW_ROTATE_BLOCK_RIGHT = 0.65;
    private static final double CLAW_ROTATE_BLOCK_DROP = 0.99;
    private static final double CLAW_SERVO_DROP = 0;
    private static final double CLAW_SERVO_PICKUP = 0.5;

    @Override
    public void runOpMode() {
        // Initialize robot hardware
        robot.initHardware(hardwareMap);
        robot.bucketServo.setPosition(START_BUCKET);
        robot.clawRotate.setPosition(CLAW_ROTATE_BLOCK_DROP);

        // Initialize GoBilda Pinpoint
        pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
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

        // Execute movements sequentially
        moveToPosition(TARGET_POSITIONX1, TARGET_POSITIONY1);
        liftToHeight(TARGET_LIFT_HEIGHT);
        dropBucket();
        moveToPosition(TARGET_POSITIONX2, TARGET_POSITIONY2);
        liftToHeight(TARGET_LIFT_HEIGHT_DOWN);
        grabBlock();
        moveToPosition(TARGET_POSITIONX3, TARGET_POSITIONY3);
        liftToHeight(TARGET_LIFT_HEIGHT);
        dropBucket();
    }

    private void moveToPosition(double targetX, double targetY) {
        double targetHeading = pinpointDriver.getHeading();
        double Kp = 0.02;

        while (opModeIsActive()) {
            pinpointDriver.update();
            Pose2D pose = pinpointDriver.getPosition();
            double x = pose.getX(DistanceUnit.MM);
            double y = pose.getY(DistanceUnit.MM);
            double currentHeading = pinpointDriver.getHeading();
            double headingError = targetHeading - currentHeading;
            double correction = headingError * Kp;

            if (x < targetX) {
                robot.setDriverMotorPower(0.4 - correction, 0.4 + correction, 0.4 - correction, 0.4 + correction);
            } else if (y < targetY) {
                robot.setDriverMotorPower(0.5 - correction, -0.5 + correction, -0.5 - correction, 0.5 + correction);
            } else {
                robot.setDriverMotorPower(0, 0, 0, 0);
                break;
            }

            telemetry.addData("X (mm)", x);
            telemetry.addData("Y (mm)", y);
            telemetry.addData("Target Heading", targetHeading);
            telemetry.addData("Current Heading", currentHeading);
            telemetry.update();
        }
    }

    private void liftToHeight(double targetHeight) {
        robot.ArmOne.setPower(-0.7);  // Reduced arm power
        robot.ArmTwo.setPower(-0.7);
        sleep(400);  // Increased sleep time
        robot.ArmOne.setPower(0.05);
        robot.ArmTwo.setPower(0.05);
        sleep(100);
        robot.liftLeft.setPower(-0.4);
        robot.liftRight.setPower(-0.4);

        while (opModeIsActive() && Math.abs(robot.liftLeft.getCurrentPosition()) < targetHeight) {
            telemetry.addData("Lift Height", robot.liftLeft.getCurrentPosition());
            telemetry.update();
        }

        robot.liftLeft.setPower(0.05);
        robot.liftRight.setPower(0.05);
        sleep(300);
    }

    private void dropBucket() {
        robot.bucketServo.setPosition(DROP_BUCKET);
        sleep(1000);
    }

    private void grabBlock() {
        // Lower the arm slowly
        robot.ArmOne.setPower(-0.1);  // Reduced power for slower descent
        robot.ArmTwo.setPower(-0.1);

        while (opModeIsActive() && !robot.touchgrab.isPressed()) {
            telemetry.addData("Arm Status", "Lowering");
            telemetry.update();
        }

        robot.ArmOne.setPower(0);
        robot.ArmTwo.setPower(0);
        telemetry.addData("Arm Status", "Stopped");
        telemetry.update();

        // Close the claw
        robot.clawServo.setPower(CLAW_CLOSE);
        telemetry.addData("Claw Status", "Closed");
        telemetry.update();
        sleep(500);

        // Raise the arm
        while (opModeIsActive() && !robot.touchdrop.isPressed()) {
            robot.ArmOne.setPower(0.2);
            robot.ArmTwo.setPower(0.2);
            robot.clawRotate.setPosition(CLAW_ROTATE_BLOCK_RIGHT);
            telemetry.addData("Arm Status", "Raising");
            telemetry.update();
        }

        robot.ArmOne.setPower(0);
        robot.ArmTwo.setPower(0);
        telemetry.addData("Arm Status", "Stopped");
        telemetry.update();

        // Open the claw
        robot.clawRotate
                .setPosition(CLAW_SERVO_DROP);
        telemetry.addData("Claw Status", "Opened");
        telemetry.update();
        sleep(1000);
    }

    private void sleep(int milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}