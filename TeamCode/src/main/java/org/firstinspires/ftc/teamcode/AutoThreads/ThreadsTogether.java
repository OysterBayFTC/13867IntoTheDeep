package org.firstinspires.ftc.teamcode.AutoThreads;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Autonomous (name = "ThreadsTogether", group = "Autonomous")
public class ThreadsTogether extends LinearOpMode {

    // Declare hardware components
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
    // TouchSensor touchdrop Digital ControlHub 2-3
    // TouchSensor touchdrop Analog ControlHub 2-3
    public Servo bucketServo;
    // Servo bucketServo ControlHub Slot2
    public CRServo clawServo;
    public Servo clawRotate;
    //... other hardware components

    private Telemetry telemetry;

    public void Threads(String threadName, String threadName2) {
        Thread thread1 = new Thread(new Runnable() {
            @Override
            public void run() {
                Thread1Code thread1Code = new Thread1Code(hardwareMap, telemetry);
                thread1Code.runOpMode();
            }
        });
        thread1.setName(threadName);

        Thread thread2 = new Thread(new Runnable() {
            @Override
            public void run() {
                Thread2Code thread2Code = new Thread2Code(hardwareMap, telemetry);
                thread2Code.runOpMode();
            }
        });
        thread2.setName(threadName2);

        thread1.start();
        thread2.start();
    } public void runThread(String threadName) {
        Thread thread1 = new Thread(new Runnable() {
            @Override
            public void run() {
                Thread1Code thread1Code = new Thread1Code(hardwareMap, telemetry);
                thread1Code.runOpMode();
            }
        });
        thread1.setName(threadName);

        thread1.start();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        motorFrontLeft = hardwareMap.get(DcMotorEx.class, "motorFrontLeft");//
        motorFrontRight = hardwareMap.get(DcMotorEx.class, "motorFrontRight"); //
        motorBackLeft = hardwareMap.get(DcMotorEx.class, "motorBackLeft"); //
        motorBackRight = hardwareMap.get(DcMotorEx.class, "motorBackRight"); //
        liftLeft = hardwareMap.get(DcMotor.class, "liftLeft"); //
        liftRight = hardwareMap.get(DcMotor.class, "liftRight"); //
        ArmOne = hardwareMap.get(DcMotor.class, "ArmOne"); //
        ArmTwo = hardwareMap.get(DcMotor.class, "ArmTwo"); //
        bucketServo = hardwareMap.get(Servo.class, "bucketServo"); //
        clawServo = hardwareMap.get(CRServo.class, "clawServo"); //
        touchdrop = hardwareMap.get(TouchSensor.class, "touchdrop"); //
        touchgrab = hardwareMap.get(TouchSensor.class, "touchgrab"); //
        clawRotate = hardwareMap.get(Servo.class, "clawRotate");
        //... initialize other hardware components

        // Set motor directions and modes
        //...

        waitForStart();


        //Threads("MovementTask2", "LiftTask2");
        runThread("MovementTask2");
    }
}