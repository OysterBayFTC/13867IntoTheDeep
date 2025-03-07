package org.firstinspires.ftc.teamcode.Base;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class AutoRobotStruct {
    // Hardware declarations
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
    public TouchSensor touchbucket, touchgrab, touchdrop;
    // TouchSensor touchgrab Digital ControlHub 2-3
    // TouchSensor touchdrop Analog ControlHub 0-1
    public Servo bucketServo;
    // Servo bucketServo ControlHub Slot
    public CRServo clawServo;
    // Servo clawServo ExpansionHub Slot3

    public Servo ServoPush;
    // Servo servoPush
    //

    public Servo clawRotate;
    // Servo clawServo ExpansionHub Slot2
    public DigitalChannel sampleSwitch;
    // Digtial Channel sampleSwitch ExpansionHub Slot 7



    // Initialize hardware
   public void initHardware(HardwareMap hardwareMap) {
    motorFrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft");// 
    motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight"); //
    motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft"); //
    motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight"); //
    liftLeft = hardwareMap.get(DcMotor.class, "liftLeft"); //

    liftRight = hardwareMap.get(DcMotor.class, "liftRight"); //
    ArmOne = hardwareMap.get(DcMotor.class, "ArmOne"); // 
    ArmTwo = hardwareMap.get(DcMotor.class, "ArmTwo"); //
    bucketServo = hardwareMap.get(Servo.class, "bucketServo"); //
    clawServo = hardwareMap.get(CRServo.class, "clawServo"); //
       ServoPush = hardwareMap.get(Servo.class, "ServoPush" ); //
               touchdrop = hardwareMap.get(TouchSensor.class, "touchdrop"); //
    touchgrab = hardwareMap.get(TouchSensor.class, "touchgrab"); //
       clawRotate = hardwareMap.get(Servo.class, "clawRotate");
       sampleSwitch = hardwareMap.get(DigitalChannel.class, "sampleSwitch");
       sampleSwitch.setMode(DigitalChannel.Mode.INPUT);
       //  touchbucket = hardwareMap.get(TouchSensor.class, "touchbucket"); //

    // Motor directions (if required)
    motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
    motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);

    // Reverse the arm motors if needed
    ArmOne.setDirection(DcMotor.Direction.REVERSE); // Reverse ArmOne
    ArmTwo.setDirection(DcMotor.Direction.REVERSE); // Reverse ArmTwo
    
     // reversed direction to keep both postive 
    liftRight.setDirection(DcMotor.Direction.REVERSE); // Reverse ArmTwo
    
   
}

    // Utility method to set motor power
    public void setDriverMotorPower(double frontRight, double frontLeft, double backRight, double backLeft) {
        motorFrontRight.setPower(frontRight);
        motorFrontLeft.setPower(frontLeft);
        motorBackRight.setPower(backRight);
        motorBackLeft.setPower(backLeft);
    }

}
