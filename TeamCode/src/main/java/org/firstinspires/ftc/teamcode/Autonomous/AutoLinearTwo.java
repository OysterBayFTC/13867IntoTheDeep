package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.teamcode.Base.AutoRobotStruct;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutoLinearTwo", group = "Autonomous")
public class AutoLinearTwo extends LinearOpMode {

    private AutoRobotStruct robot = new AutoRobotStruct();
    private GoBildaPinpointDriver pinpointDriver;

    private static final double TARGET_POSITIONX1 = 125.0;
    private static final double TARGET_POSITIONY1 = -515.0;
    private static final double MIDDLE_BLOCK_PICKUP_X2 = 205;
    private static final double MIDDLE_BLOCK_PICKUP_Y2 = -470;
    private static final double TARGET_LIFT_HEIGHT = 5500;
    private static final double TARGET_LIFT_HEIGHT_DOWN = 900;
    private static final double SCORE_POSITION_X1 = 130;
    private static final double SCORE_POSITION_Y1 = -525;

    @Override
    public void runOpMode() {
        // Initialize robot hardware
        robot.initHardware(hardwareMap);
        robot.bucketServo.setPosition(0.5);

        // Initialize GoBilda Pinpoint
        pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        pinpointDriver.setOffsets(-84.0, -168.0);
        pinpointDriver.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpointDriver.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpointDriver.resetPosAndIMU();

        // Initialize lift motors with encoders
        robot.liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            performLift(TARGET_LIFT_HEIGHT, 0.75);
            moveToPosition(TARGET_POSITIONX1, TARGET_POSITIONY1);

            robot.bucketServo.setPosition(0.15);
            sleep(600);
            robot.bucketServo.setPosition(0.50);

            performLift(TARGET_LIFT_HEIGHT_DOWN, -0.5);
            moveToPosition(MIDDLE_BLOCK_PICKUP_X2, MIDDLE_BLOCK_PICKUP_Y2);
            grabBlock();

            performLift(TARGET_LIFT_HEIGHT, 0.75);
            moveToPosition(SCORE_POSITION_X1, SCORE_POSITION_Y1);

            robot.bucketServo.setPosition(0.15);
            sleep(600);
            robot.bucketServo.setPosition(0.50);
        }
    }

    private void performLift(double targetHeight, double power) {
        robot.liftLeft.setPower(power);
        robot.liftRight.setPower(-power);

        while (opModeIsActive() && Math.abs(robot.liftLeft.getCurrentPosition()) < targetHeight) {
            telemetry.addData("Lift Left Encoder", robot.liftLeft.getCurrentPosition());
            telemetry.addData("Lift Right Encoder", robot.liftRight.getCurrentPosition());
            telemetry.addData("Target Height", targetHeight);
            telemetry.update();
        }

        robot.liftLeft.setPower(0.1 * Math.signum(power));
        robot.liftRight.setPower(-0.1 * Math.signum(power));
        sleep(300);
    }

    private void moveToPosition(double targetX, double targetY) {
        while (opModeIsActive()) {
            pinpointDriver.update();
            Pose2D pose = pinpointDriver.getPosition();
            double x = pose.getX(DistanceUnit.MM);
            double y = pose.getY(DistanceUnit.MM);

            if (x < targetX) {
                robot.setDriverMotorPower(0.4, 0.4, 0.4, 0.4); // Move forward
            } else if (y > targetY) {
                robot.setDriverMotorPower(0.5, -0.5, -0.5, 0.5); // Strafe right
            } else {
                robot.setDriverMotorPower(0, 0, 0, 0);
                break;
            }

            telemetry.addData("X (mm)", x);
            telemetry.addData("Y (mm)", y);
            telemetry.update();
        }
    }

    private void grabBlock() {
        while (opModeIsActive() && !robot.touchgrab.isPressed()) {
            robot.ArmOne.setPower(-0.2);
            robot.ArmTwo.setPower(-0.2);
        }
        robot.ArmOne.setPower(0);
        robot.ArmTwo.setPower(0);
        sleep(1000);

        while (opModeIsActive() && !robot.touchdrop.isPressed()) {
            robot.ArmOne.setPower(0.2);
            robot.ArmTwo.setPower(0.2);
        }
        robot.ArmOne.setPower(0);
        robot.ArmTwo.setPower(0);
    }

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