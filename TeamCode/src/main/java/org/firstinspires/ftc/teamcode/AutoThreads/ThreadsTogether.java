package org.firstinspires.ftc.teamcode.AutoThreads;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Base.AutoRobotStruct;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

@Autonomous(name = "ThreadsTogether", group = "Autonomous")
public class ThreadsTogether extends LinearOpMode {
    private AutoRobotStruct robot = new AutoRobotStruct(); // Hardware structure
    private GoBildaPinpointDriver pinpointDriver;
    private volatile boolean movementComplete1 = false;
    private volatile boolean movementComplete2b = false;

    // Constants (Lift heights, target positions, servo positions etc.)
    private static final double InitalTargetx = 100;
    private static final double InitalTargety = 540.0;
    private static final double MIddleBlockx  = 415;//390
    private static final double MiddleBlockY  = -485;
    private static final double MiddleBlockY2 = -505;
    private static final double RightBlockx  = 405;//380
    private static final double RightBlocky  = -230;
    private static final double LiftHeightUp  = 5700; // height for the lift
    private static final double LiftHeightoDown = 100; // height for the lift
    private static final double Scorepositionx1  = 325; // score position X
    private static final double ScorePositiony1  = -550; // score position Y
    private static final double ScorePositoinx2  = 295; // score position X
    private static final double Scorepositiony2  = -590; // score position Y
    private static final double ParkPostionx  = 1200; // park position X
    private static final double ParkPostiony  = 1200; // park position Y
    private static final double dropBucket = 0.00;
    private static final double neutralBucket = 0.35; // Catch position
    private static final double startBucket = 0.45; // Catch position
    private static final double clawOpen = 0.65;
    private static final double clawClose = 0.45;
    // these are the values for the rotation for the claw servo
    double clawRotateBlockLeft = 0;
    double clawRotateBlockRight = .65;
    double clawRotateBlockVert = .35;
    double clawRotateBlockDrop = .99;
    private static final double clawServoDrop = -1; // score position X
    private static final double clawServoPickup = 1; // score position X

    // Declare hardware components
    public DcMotor motorFrontRight, motorFrontLeft, motorBackRight, motorBackLeft;
    public DcMotor ArmOne, ArmTwo, liftLeft, liftRight;
    public TouchSensor touchbucket, touchgrab, touchdrop;
    public Servo bucketServo;
    public CRServo clawServo;
    public Servo clawRotate;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        motorFrontLeft = hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
        motorFrontRight = hardwareMap.get(DcMotorEx.class, "motorFrontRight");
        motorBackLeft = hardwareMap.get(DcMotorEx.class, "motorBackLeft");
        motorBackRight = hardwareMap.get(DcMotorEx.class, "motorBackRight");
        liftLeft = hardwareMap.get(DcMotor.class, "liftLeft");
        liftRight = hardwareMap.get(DcMotor.class, "liftRight");
        ArmOne = hardwareMap.get(DcMotor.class, "ArmOne");
        ArmTwo = hardwareMap.get(DcMotor.class, "ArmTwo");
        bucketServo = hardwareMap.get(Servo.class, "bucketServo");
        clawServo = hardwareMap.get(CRServo.class, "clawServo");
        touchdrop = hardwareMap.get(TouchSensor.class, "touchdrop");
        touchgrab = hardwareMap.get(TouchSensor.class, "touchgrab");
        clawRotate = hardwareMap.get(Servo.class, "clawRotate");
        robot.initHardware(hardwareMap);
        robot.bucketServo.setPosition(startBucket); // Set bucketServo to start position
        robot.clawRotate.setPosition(clawRotateBlockDrop);

        // Initialize GoBilda Pinpoint
        pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // Set up Pinpoint driver configuration
        pinpointDriver.setOffsets(-84.0, -168.0);
        pinpointDriver.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpointDriver.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpointDriver.resetPosAndIMU();

        // Initialize encoders for arm motors

        // for lifts
        robot.liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // for wheels
        robot.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // for arms
        robot.ArmOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.ArmTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        // Run MovementTask1 and LiftTask1 together
        runThreads("MovementTask1", "LiftTask1");
        sleep(150);
        // Run LimitDown2 and ForwardMovement2 together
        runThreads("LimitDown2", "ForwardMovement2");
        telemetry.addData("Threads", "Finished (check telemetry for which threads ran)");
        telemetry.update();




        // Run ForwardMovement3 and LiftTask3 together
        runThreads("ForwardMovement3", "LiftTask3");

        // Run ForwardMovement4 and LiftTask3 together
        runThreads("LimitDown4", "ForwardMovement4");

        // Run ForwardMovement5 and LiftTask5 together
        runThreads("ForwardMovement5", "LiftTask5");

        // Run ForwardMovement6 and LiftTask6 together
        runThreads("LimitDown6", "ForwardMovement6");


        telemetry.addData("Threads", "Finished (check telemetry for which threads ran)");
        telemetry.update();
        sleep(2000); // Keep telemetry visible for a moment
    }

    // Function to run a single thread by name
    public void runThread(String threadName) {
        Thread thread = null;
        switch (threadName) {
            case "MovementTask1":
                thread = new Thread(new ThreadTasks1.MovementTask1(robot, pinpointDriver, telemetry, this));
                break;
            case "LiftTask1":
                thread = new Thread(new ThreadTasks1.LiftTask1(robot, telemetry, this));
                break;
            case "LimitDown2":
                thread = new Thread(new ThreadTasks2.LimitDown2(robot, telemetry, this));
                break;
            case "ForwardMovement2":
                thread = new Thread(new ThreadTasks2.ForwardMovement2(robot, pinpointDriver, telemetry, this));
                break;
            case "ForwardMovement3":
                thread = new Thread(new ThreadTasks3.ForwardMovement3(robot, pinpointDriver, telemetry, this));
                break;


            default:
                telemetry.addData("Error", "Unknown thread name: " + threadName);
                telemetry.update();
                return; // Don't start thread if name is unknown
        }

        if (thread != null) {
            thread.setName(threadName);
            thread.start();
            try {
                thread.join(); // Wait for thread to finish
            } catch (InterruptedException e) {
                telemetry.addData("Error", "Thread interrupted: " + threadName + " - " + e.getMessage());
                telemetry.update();
                Thread.currentThread().interrupt(); // Re-interrupt current thread
            }
        }
    }

    // Function to run two threads together by name
    public void runThreads(String threadName1, String threadName2) {
        Thread thread1 = null;
        Thread thread2 = null;

        switch (threadName1) {
            case "MovementTask1":
                thread1 = new Thread(new ThreadTasks1.MovementTask1(robot, pinpointDriver, telemetry, this));
                break;
            case "LiftTask1":
                thread1 = new Thread(new ThreadTasks1.LiftTask1(robot, telemetry, this));
                break;
            case "LimitDown2":
                thread1 = new Thread(new ThreadTasks2.LimitDown2(robot, telemetry, this));
                break;
            case "ForwardMovement2":
                thread1 = new Thread(new ThreadTasks2.ForwardMovement2(robot, pinpointDriver, telemetry, this));
                break;
            case "ForwardMovement3":
                thread1 = new Thread(new ThreadTasks3.ForwardMovement3(robot, pinpointDriver, telemetry, this));
                break;
            case "LiftTask3":
                thread1 = new Thread(new ThreadTasks3.LiftTask3(robot, telemetry, this));
                break;
            case "LimitDown4":
                thread1 = new Thread(new ThreadTasks4.LimitDown4(robot, telemetry, this));
                break;
            case "ForwardMovement4":
                thread1 = new Thread(new ThreadTasks4.ForwardMovement4(robot, pinpointDriver, telemetry, this));
                break;
            case "ForwardMovement5":
                thread1 = new Thread(new ThreadTasks5.ForwardMovement5(robot, pinpointDriver, telemetry, this));
                break;
            case "LiftTask5":
                thread1 = new Thread(new ThreadTasks5.LiftTask5(robot, telemetry, this));
                break;

            default:
                telemetry.addData("Error", "Unknown thread name: " + threadName1);
                telemetry.update();
                return; // Don't start threads if name is unknown
        }

        switch (threadName2) {
            case "MovementTask1":
                thread2 = new Thread(new ThreadTasks1.MovementTask1(robot, pinpointDriver, telemetry, this));
                break;
            case "LiftTask1":
                thread2 = new Thread(new ThreadTasks1.LiftTask1(robot, telemetry, this));
                break;
            case "LimitDown2":
                thread2 = new Thread(new ThreadTasks2.LimitDown2(robot, telemetry, this));
                break;
            case "ForwardMovement2":
                thread2 = new Thread(new ThreadTasks2.ForwardMovement2(robot, pinpointDriver, telemetry, this));
                break;
            case "ForwardMovement3":
                thread2 = new Thread(new ThreadTasks3.ForwardMovement3(robot, pinpointDriver, telemetry, this));
                break;
            case "LiftTask3":
                thread2 = new Thread(new ThreadTasks3.LiftTask3(robot, telemetry, this));
                break;
            case "LimitDown4":
                thread2 = new Thread(new ThreadTasks4.LimitDown4(robot, telemetry, this));
                break;
            case "ForwardMovement4":
                thread2 = new Thread(new ThreadTasks4.ForwardMovement4(robot, pinpointDriver, telemetry, this));
                break;
            case "ForwardMovement5":
                thread2 = new Thread(new ThreadTasks5.ForwardMovement5(robot, pinpointDriver, telemetry, this));
                break;
            case "LiftTask5":
                thread2 = new Thread(new ThreadTasks5.LiftTask5(robot, telemetry, this));
                break;

            default:
                telemetry.addData("Error", "Unknown thread name: " + threadName2);
                telemetry.update();
                return; // Don't start threads if name is unknown
        }

        if (thread1 != null && thread2 != null) {
            thread1.setName(threadName1);
            thread2.setName(threadName2);
            thread1.start();
            thread2.start();

            try {
                thread1.join(); // Wait for both threads to finish
                thread2.join();
            } catch (InterruptedException e) {
                telemetry.addData("Error", "Thread interrupted: " + threadName1 + " or " + threadName2 + " - " + e.getMessage());
                telemetry.update();
                Thread.currentThread().interrupt(); // Re-interrupt current thread
            }
        }
    }

    // Helper method to sleep without exception interruptions (in main OpMode now)
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