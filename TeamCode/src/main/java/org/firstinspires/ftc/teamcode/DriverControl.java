package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Base.RobotStructure;

@TeleOp(name = "DriverControl")
public class DriverControl extends RobotStructure {
    private final boolean motorState = false;
    double clawServoPosition = 0.15; // Initial position
    double clawRotateBlockLeft = 0;
    double clawRotateBlockRight = .65;
    double clawRotateBlockVert = .35;
    double clawRotateBlockDrop = .99;
    private final double clawStepUp = 0.20;
    private final double clawStepDown = .20;
    private double clawPosition = 0.0;
    boolean xButtonPressed = false; // To track if X button is pressed
    boolean aButtonPressed = false; // To track if a button is pressed
    private boolean prevA = false;
    private boolean prevB = false;
    private boolean prevX = false;
    @Override
    public void init() {
        super.init(); // Ensure RobotStructure's initialization happens
        boolean touchdropState = touchdrop.isPressed();

        telemetry.addData("Touchdrop Sensor State", touchdropState ? "Pressed" : "Released");
        telemetry.addData("Touchgrab Sensor State", touchdropState ? "Pressed" : "Released");

        telemetry.update();
        ArmOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize encoders for arm motors
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Set it to neutral (stop)
        clawServo.setPosition(0.0);

    }

    @Override
    public void loop() {
        // Run motor and servo control logic
        initDriver();
        controlMotors();
        controlServos();
        // updateTelemetry();
        telemetry.addData("Target Claw Servo ", clawPosition);
        telemetry.addData("Actual Claw Servo ", clawServo.getPosition());
        telemetry.update();
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


    private void controlMotors() {
        // Arm control logic

        double triggerPowerRight = gamepad2.right_trigger * .45;
        double triggerPowerLeft = gamepad2.left_trigger * .45;

        if (triggerPowerRight > 0 && !touchgrab.isPressed()) {
            ArmOne.setPower(-triggerPowerRight); // Move towards Grab
            ArmTwo.setPower(-triggerPowerRight);
        } else if (triggerPowerLeft > 0 && !touchdrop.isPressed()) {
            ArmOne.setPower(triggerPowerLeft); // Move towards Bucket
            ArmTwo.setPower(triggerPowerLeft);
        }
        else if (triggerPowerRight > 0 && gamepad2.y) {
            ArmOne.setPower(-triggerPowerRight); // Move towards Grab
            ArmTwo.setPower(-triggerPowerRight);
        } else if (triggerPowerLeft > 0 && gamepad2.y) {
            ArmOne.setPower(triggerPowerLeft); // Move towards Bucket
            ArmTwo.setPower(triggerPowerLeft);
        }
        else{
            // If neither bumper is pressed, stop the arm motors
            ArmOne.setPower(0.1);
            ArmTwo.setPower(-0.1);
        }

        // Lift motors control
        int liftLimitHeightUp = 5400;
        int liftLimitHeightDown = 900;
        if (gamepad2.dpad_up /*&& (Math.abs(liftLeft.getCurrentPosition()) < liftLimitHeightUp)*/) {
            liftLeft.setPower(-0.75);
            liftRight.setPower(-0.75);
        } else if (gamepad2.dpad_down /*&& (Math.abs(liftLeft.getCurrentPosition()) > liftLimitHeightDown)*/) {
            liftLeft.setPower(0.75);
            liftRight.setPower(0.75);
        } else if (gamepad2.dpad_right) {
            liftLeft.setPower(-0.01);
            liftRight.setPower(-0.01);
        } else if (gamepad2.dpad_left) {
            liftLeft.setPower(-0.9);
            liftRight.setPower(-0.9);
        } else {
            liftLeft.setPower(0.);
            liftRight.setPower(0.);
        }
    }

    private void controlServos() {

        boolean currA = gamepad2.a;
        boolean currB = gamepad2.b;
        boolean currX = gamepad2.x;


        if (currA && !prevA) {
            clawPosition = 0.0;
            clawServo.setPosition(clawPosition);
        }

        if (currB && !prevB) {
            if ((clawPosition + clawStepUp) <= 1.0) {
                clawPosition += clawStepUp;
                clawServo.setPosition(clawPosition);
            }
        }

        if (currX && !prevX) {
            if ((clawPosition - clawStepUp) >= 0.0) {
                clawPosition -= clawStepDown;
                clawServo.setPosition(clawPosition);
            }
        }
        prevA = currA;
        prevB = currB;
        prevX = currX;

        //  Code for Claw Rotate
        if (gamepad1.x) {
            clawRotate.setPosition(clawRotateBlockLeft); // When block is left of claw
        }
        else   if (gamepad1.a) {
            clawRotate.setPosition(clawRotateBlockVert); // When block is vertical
        }
        else   if (gamepad1.b) {
            clawRotate.setPosition(clawRotateBlockRight); // Whem block is right of claw
        }
        else   if (gamepad1.y) {
            clawRotate.setPosition(clawRotateBlockDrop); // Whem block is right of claw
        }


        // Bucket servo control
        if (gamepad1.left_bumper) {
            bucketServo.setPosition(0.005); // Set to score position
        }
        else if (gamepad1.right_bumper) {
            bucketServo.setPosition(0.3); // Set to drop  position
        }
        //      else if (gamepad1.a) {
        //        bucketServo.setPosition(0.45); // used to move the bucket out of the way
        //  }
    }

    private void updateTelemetry() {
        boolean touchStateDrop = !touchdrop.isPressed(); // Get touchdrop sensor state
        telemetry.addData("Drop Sensor", touchStateDrop ? "Released" : "Pressed");

        boolean touchStateGrab = !touchgrab.isPressed(); // Get touchgrab sensor state
        telemetry.addData("Grab Sensor", touchStateGrab ? "Released" : "Pressed");

        telemetry.update(); // Update telemetry display
    }
}