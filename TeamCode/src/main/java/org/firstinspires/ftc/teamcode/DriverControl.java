package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Base.RobotStructure;

@TeleOp(name = "DriverControl")
public class DriverControl extends RobotStructure {
    private boolean motorState = false;
    double clawServoPosition = 0.15; // Initial position
    boolean xButtonPressed = false; // To track if X button is pressed
    boolean aButtonPressed = false; // To track if a button is pressed

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
        clawServo.setPosition(0.5);

            }

    @Override
    public void loop() {
        // Run motor and servo control logic
        initDriver();
        controlMotors();
        controlServos();
        updateTelemetry();
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
          if (gamepad2.right_bumper && !touchgrab.isPressed()) {
    ArmOne.setPower(-0.3); // Move towards Grab
    ArmTwo.setPower(-0.3);
} else if (gamepad2.left_bumper && !touchdrop.isPressed()) {
    ArmOne.setPower(0.3); // Move towards Bucket
    ArmTwo.setPower(0.3);
} 
 else if (gamepad2.right_bumper && gamepad2.y) {
    ArmOne.setPower(-0.3); // Move towards Grab
    ArmTwo.setPower(-0.3);
} else if (gamepad2.left_bumper && gamepad2.y) {
    ArmOne.setPower(0.3); // Move towards Bucket
    ArmTwo.setPower(0.3);
}
else{
    // If neither bumper is pressed, stop the arm motors
    ArmOne.setPower(0.1);
    ArmTwo.setPower(-0.1);
}

        // Lift motors control
        int liftLimitHeightUp = 5400;
        int liftLimitHeightDown = 900;
        if (gamepad2.dpad_up && (Math.abs(liftLeft.getCurrentPosition()) < liftLimitHeightUp)) {
            liftLeft.setPower(0.75);
            liftRight.setPower(0.75);
        } else if (gamepad2.dpad_down && (Math.abs(liftLeft.getCurrentPosition()) > liftLimitHeightDown)) {
            liftLeft.setPower(-0.75);
            liftRight.setPower(-0.75);
        } else if (gamepad2.dpad_right) {
            liftLeft.setPower(0.1);
            liftRight.setPower(0.1);
        } else if (gamepad2.dpad_left) {
            liftLeft.setPower(-1);
            liftRight.setPower(-1);
        } else {
            liftLeft.setPower(0.);
            liftRight.setPower(0.);
        }
    }

    private void controlServos() {


        if (gamepad2.a) {
            if (!aButtonPressed) { // Only execute once per button press
                aButtonPressed = true; // Mark button as pressed
                clawServoPosition += 0.33; // Decrease position by 0.05
                if (clawServoPosition > 0.99) {
                    clawServoPosition = 0.99; // Ensure position doesn't go below 0
                }
                clawServo.setPosition(clawServoPosition); // Update servo position
            }
        } else {
            aButtonPressed = false; // Reset button state when not pressed
        }
        if (gamepad2.b) {
            clawServo.setPosition(0.00); //  setting the postion to 0, which is equal to 0 degrees.
        }
        // Check if X button is pressed
        if (gamepad2.x) {
            if (!xButtonPressed) { // Only execute once per button press
                xButtonPressed = true; // Mark button as pressed
                clawServoPosition -= 0.05; // Decrease position by 0.05
                if (clawServoPosition < 0.0) {
                    clawServoPosition = 0.0; // Ensure position doesn't go below 0
                }
                clawServo.setPosition(clawServoPosition); // Update servo position
            }
        } else {
            xButtonPressed = false; // Reset button state when not pressed
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
