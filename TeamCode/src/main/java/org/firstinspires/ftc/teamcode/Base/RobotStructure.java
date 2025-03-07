package org.firstinspires.ftc.teamcode.Base;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class RobotStructure extends OpMode {

    // Hardware declarations and configurations
    public DcMotor motorFrontRight, motorFrontLeft, motorBackRight, motorBackLeft;
    public DcMotor ArmOne, ArmTwo, liftLeft, liftRight;

    public TouchSensor touchdrop, touchgrab;
    public DigitalChannel sampleSwitch;

    public Servo bucketServo;
    public CRServo clawServo;
    public Servo clawRotate;

    // Add webcam declaration
    public WebcamName webcam; // Corrected variable type

    @Override
    public void init() {
        // Initialize hardware mappings
        motorFrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");

        liftLeft = hardwareMap.get(DcMotor.class, "liftLeft");
        liftRight = hardwareMap.get(DcMotor.class, "liftRight");

        ArmOne = hardwareMap.get(DcMotor.class, "ArmOne");
        ArmTwo = hardwareMap.get(DcMotor.class, "ArmTwo");

        bucketServo = hardwareMap.get(Servo.class, "bucketServo");
        clawServo = hardwareMap.get(CRServo.class, "clawServo");
        clawRotate = hardwareMap.get(Servo.class, "clawRotate");

        touchdrop = hardwareMap.get(TouchSensor.class, "touchdrop");
        touchgrab = hardwareMap.get(TouchSensor.class, "touchgrab");

        sampleSwitch = hardwareMap.get(DigitalChannel.class, "sampleSwitch");
        sampleSwitch.setMode(DigitalChannel.Mode.INPUT);

        // Webcam initialization
        webcam = hardwareMap.get(WebcamName.class, "Webcam"); // Corrected webcam mapping

        // Set motor directions
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);

        ArmOne.setDirection(DcMotor.Direction.REVERSE);
        ArmTwo.setDirection(DcMotor.Direction.REVERSE);

        liftRight.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Robot Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Empty loop; actual logic is in derived classes
    }

    public void setDriverMotorPower(double FRightPower, double FLeftPower, double BRightPower, double BLeftPower) {
        motorFrontRight.setPower(FRightPower);
        motorFrontLeft.setPower(FLeftPower);
        motorBackRight.setPower(BRightPower);
        motorBackLeft.setPower(BLeftPower);
    }
}
