package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Base.RobotStructure;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

@TeleOp(name = "DriverControlWithOdometry")
public class DriverControlWithOdometry extends RobotStructure {
    private boolean positionSaved = false;
    private Pose2D savedPosition;
    private GoBildaPinpointDriver pinpointDriver;

    @Override
    public void init() {
        super.init(); // Ensure RobotStructure's initialization happens

        // Initialize GoBilda Pinpoint Driver for odometry
        pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        pinpointDriver.setOffsets(-84.0, -168.0);
        pinpointDriver.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpointDriver.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpointDriver.resetPosAndIMU();

        telemetry.addData("Odometry", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        pinpointDriver.update(); // Refresh odometry data

        // Handle driver controls
        initDriver();
        controlMotors();
        controlServos();
        odometryControls();
        updateTelemetry();
    }

    private void initDriver() {
        float gamepad1LeftY = gamepad1.left_stick_y;
        float gamepad1LeftX = gamepad1.left_stick_x;
        float gamepad1RightX = -gamepad1.right_stick_x;

        float translation = -gamepad1LeftX;
        float rotation = gamepad1RightX;

        float frontRightPower = -gamepad1LeftY + translation + rotation;
        float frontLeftPower = -gamepad1LeftY - translation - rotation;
        float backLeftPower = -gamepad1LeftY + translation - rotation;
        float backRightPower = -gamepad1LeftY - translation + rotation;

        motorFrontLeft.setPower(-frontLeftPower / 1.5);
        motorBackLeft.setPower(-backLeftPower / 1.5);
        motorFrontRight.setPower(-frontRightPower / 1.5);
        motorBackRight.setPower(-backRightPower / 1.5);
    }

    private void controlMotors() {
        if (gamepad2.right_bumper && !touchgrab.isPressed()) {
            ArmOne.setPower(-0.3);
            ArmTwo.setPower(-0.3);
        } else if (gamepad2.left_bumper && !touchdrop.isPressed()) {
            ArmOne.setPower(0.3);
            ArmTwo.setPower(0.3);
        } else {
            ArmOne.setPower(0);
            ArmTwo.setPower(0);
        }

        if (gamepad2.dpad_up) {
            liftLeft.setPower(-0.75);
            liftRight.setPower(0.75);
        } else if (gamepad2.dpad_down) {
            liftLeft.setPower(0.75);
            liftRight.setPower(-0.75);
        } else {
            liftLeft.setPower(0);
            liftRight.setPower(0);
        }
    }

    private void controlServos() {
        if (gamepad2.a) {
            clawServo.setPosition(0.39);
        }
        if (gamepad2.b) {
            clawServo.setPosition(0.9);
        }

        if (gamepad1.left_bumper) {
            bucketServo.setPosition(0.07);
        } else if (gamepad1.right_bumper) {
            bucketServo.setPosition(0.3);
        }
    }

    private void odometryControls() {
        // Save the current position when the left trigger is pressed
        if (gamepad1.left_trigger > 0.5 && !positionSaved) {
            savedPosition = pinpointDriver.getPosition();
            positionSaved = true;
            telemetry.addData("Position Saved", "X: %.2f, Y: %.2f", savedPosition.getX(DistanceUnit.MM), savedPosition.getY(DistanceUnit.MM));
            telemetry.update();
        }

        // Return to the saved position when the right trigger is pressed
        if (gamepad1.right_trigger > 0.5 && positionSaved) {
            Pose2D currentPosition = pinpointDriver.getPosition();
            double xError = savedPosition.getX(DistanceUnit.MM) - currentPosition.getX(DistanceUnit.MM);
            double yError = savedPosition.getY(DistanceUnit.MM) - currentPosition.getY(DistanceUnit.MM);

            if (Math.abs(xError) > 10) {
                double power = xError > 0 ? 0.4 : -0.4;
                motorFrontLeft.setPower(power);
                motorBackLeft.setPower(power);
                motorFrontRight.setPower(power);
                motorBackRight.setPower(power);
            } else if (Math.abs(yError) > 10) {
                double power = yError > 0 ? 0.4 : -0.4;
                motorFrontLeft.setPower(-power);
                motorBackLeft.setPower(power);
                motorFrontRight.setPower(power);
                motorBackRight.setPower(-power);
            } else {
                motorFrontLeft.setPower(0);
                motorBackLeft.setPower(0);
                motorFrontRight.setPower(0);
                motorBackRight.setPower(0);
                telemetry.addData("Position Reached", "Returning complete");
                telemetry.update();
            }
        }
    }

    private void updateTelemetry() {
        Pose2D currentPosition = pinpointDriver.getPosition();
        telemetry.addData("Current X (mm)", currentPosition.getX(DistanceUnit.MM));
        telemetry.addData("Current Y (mm)", currentPosition.getY(DistanceUnit.MM));

        telemetry.addData("Position Saved", positionSaved ? "Yes" : "No");
        if (positionSaved) {
            telemetry.addData("Saved X (mm)", savedPosition.getX(DistanceUnit.MM));
            telemetry.addData("Saved Y (mm)", savedPosition.getY(DistanceUnit.MM));
        }

        telemetry.update();
    }
}