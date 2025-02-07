package org.firstinspires.ftc.teamcode.Base;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class RobotStructure extends OpMode {
    
    // Hardware declarations and configurations 
    public DcMotor motorFrontRight, motorFrontLeft, motorBackRight, motorBackLeft;
 //   public DcMotor motorFrontRight ControlHub Slot3
 //   public DcMotor motorBackRight ControlHub Slot1
 //   public DcMotor motorFrontLeft ControlHub Slot0
 //   public DcMotor motorBackLeft ControlHub Slot2
    public DcMotor ArmOne, ArmTwo, liftLeft, liftRight;
    // DcMotor ArmOne ExpansionHub  Slot0 
    // DcMotor ArmTwo ExpansionHub  Slot3 
    // DcMotor LiftLeft ExpansionHub  Slot2
    // DcMotor LiftRight ExpansionHub Slot1 
    public TouchSensor touchdrop, touchgrab, touchbucket;
    // TouchSensor touchdrop Digital ControlHub 2-3
    // TouchSensor touchdrop Analog ControlHub 2-3
    public Servo bucketServo;
    // Servo bucketServo ControlHub Slot2
    public CRServo clawServo;
    public Servo clawRotate;
    // Servo clawServo ExpansionHub Slot2 
    
// Game pad 1 A - low bucket
// Game Pad 1 Right Bumper - Bucket Servo
// Game pad 2 B - claw servo
// Game pad 2 A - claw servo
// Game pad 2 Right Bumper - Arm Rotate Forward
// Game Pad 2 Left bumper - Arm Rotate backward
// Game pad 2 D-Pad Up - Lift Up
// Game pad 2 D-pad down - Lift down
// Game pad 2 D-pad Right - hold lift



    @Override
    public void init() {
        // Hardware mapping to the robot configuration
        motorFrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");
       // motorLiftLeft = hardwareMap.get(DcMotor.class, "motorLiftLeft");
        liftLeft = hardwareMap.get(DcMotor.class, "liftLeft");
        liftRight = hardwareMap.get(DcMotor.class, "liftRight");
        ArmOne = hardwareMap.get(DcMotor.class, "ArmOne");
        ArmTwo = hardwareMap.get(DcMotor.class, "ArmTwo");
        bucketServo = hardwareMap.get(Servo.class, "bucketServo");
        clawServo = hardwareMap.get(CRServo.class, "clawServo");
        touchdrop = hardwareMap.get(TouchSensor.class, "touchdrop");
        touchgrab = hardwareMap.get(TouchSensor.class, "touchgrab");
        clawRotate = hardwareMap.get(Servo.class, "clawRotate");

    //    touchbucket = hardwareMap.get(TouchSensor.class, "touchbucket");




        // Servo initialization
        //servoOne = hardwareMap.get(Servo.class, "servoOne");
        //servoTwo = hardwareMap.get(Servo.class, "servoTwo");

        // Touch sensor initialization

        // Motor directions (if required)
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        
        // Reverse the arm motors if needed
    ArmOne.setDirection(DcMotor.Direction.REVERSE); // Reverse ArmOne
    ArmTwo.setDirection(DcMotor.Direction.REVERSE); // Reverse ArmTwo
    
     // reversed direction to keep both postive 
    liftRight.setDirection(DcMotor.Direction.REVERSE); // Reverse ArmTwo
        
        telemetry.addData("Status", "Robot Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Empty loop as the logic is handled in derived classes like DriverControl
    }
        public void setDriverMotorPower(double FRightPower, double FLeftPower, double BRightPower, double BLeftPower) {
        motorFrontRight.setPower(FRightPower);
        motorFrontLeft.setPower(FLeftPower);
        motorBackRight.setPower(BRightPower);
        motorBackLeft.setPower(BLeftPower);
    }
    
}
