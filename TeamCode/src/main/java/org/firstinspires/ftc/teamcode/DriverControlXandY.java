package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Base.RobotStructure;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

@TeleOp(name = "DriverControlXandY")
public class DriverControlXandY extends RobotStructure {
    private boolean motorState = false;
    private GoBildaPinpointDriver pinpointDriver;  // Declare Pinpoint Driver

    @Override
    public void init() {
        super.init(); // Ensure RobotStructure's initialization happens

        pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "odo"); // Initialize the Pinpoint Driver
        pinpointDriver.setOffsets(-84.0, -168.0); // Set the offset (if required)
        pinpointDriver.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD); // Use 4-bar odometry pod
        pinpointDriver.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD); // Set directions
        pinpointDriver.resetPosAndIMU(); // Reset position and IMU for fresh odometry
        pinpointDriver.recalibrateIMU();
        ArmOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Run motor and servo control logic
        initDriver();
        controlMotors();
        controlServos();
        updateTelemetry(); // Update telemetry to show the latest data
    }

    private void initDriver() {
        float gamepad1LeftY = gamepad1.left_stick_y;
        float gamepad1LeftX = gamepad1.left_stick_x;
        float gamepad2RightY = gamepad2.right_stick_y;
        float gamepad1RightX = -gamepad1.right_stick_x;
        float gamepad2LeftY = gamepad2.left_stick_y;

        // Adjust the signs for translation
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

    // Update telemetry data to include Pinpoint X and Y position
    private void updateTelemetry() {
        boolean touchState = touchdrop.isPressed(); // Get touch sensor state
        telemetry.addData("Touch Sensor", touchState ? "Released" : "Pressed");

        // Get current position from Pinpoint Driver
        pinpointDriver.update(); // Refresh odometry data
        Pose2D pose = pinpointDriver.getPosition(); // Get position of the robot
        double x = pose.getX(DistanceUnit.MM); // Get X position in mm
        double y = pose.getY(DistanceUnit.MM); // Get Y position in mm

        // Display position data on the telemetry
        telemetry.addData("X (mm)", x);
        telemetry.addData("Y (mm)", y);

        telemetry.update(); // Update telemetry display
    }

    // Method for controlling the motors (as per the original logic)
    private void controlMotors() {
        if (gamepad1.a && !touchdrop.isPressed()) {
            ArmOne.setPower(0.45); // Forward
            ArmTwo.setPower(0.45);
        } else if (gamepad1.y) {
            ArmOne.setPower(-0.3); // Backward
            ArmTwo.setPower(-0.3);
        } else if (touchdrop.isPressed()) {
            ArmOne.setPower(0); // Stop
            ArmTwo.setPower(0);
        } else {
            ArmOne.setPower(0); // Stop
            ArmTwo.setPower(0);
        }

        // Lift motors
        if (gamepad1.x) {
            liftLeft.setPower(0.75);
            liftRight.setPower(-0.75);
        } else if (gamepad1.b) {
            liftLeft.setPower(-0.75);
            liftRight.setPower(0.75);
        } else {
            liftLeft.setPower(0);
            liftRight.setPower(0);
        }
    }

    private void controlServos() {
        if (gamepad2.a) {
            clawServo.setPosition(0.40); // Set servoTwo to 0.40 when button A is pressed
        }
        if (gamepad2.b) {
            clawServo.setPosition(0.10); // Set servoTwo to 0.10 when button B is pressed
        }
        // Code for bucket servo
        if (gamepad2.left_bumper) {
            bucketServo.setPosition(0.30); // Set bucketServo to 0.40 when button left bumper is pressed
        }
        if (gamepad2.right_bumper) {
            bucketServo.setPosition(0.5); // Set bucketServo to 0.10 when right bumper is pressed
        }
    }
}
