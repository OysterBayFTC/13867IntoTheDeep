package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Base.AutoRobotStruct;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutonomousLiftTranslate", group = "Autonomous")
public class AutonomousLiftTranslate extends LinearOpMode {

    private AutoRobotStruct robot = new AutoRobotStruct(); // Hardware structure
    private GoBildaPinpointDriver pinpointDriver;
    private volatile boolean movementComplete1 = false;
    private volatile boolean movementComplete2 = false;
    private volatile boolean movementComplete3 = false;
    private volatile boolean movementComplete4 = false;
    private volatile boolean movementComplete5 = false;
    private volatile boolean movementComplete6 = false;
    private static final double TARGET_POSITIONX1 = 100;
    private static final double TARGET_POSITIONY1 = 540.0;
    private static final double middle_block_pickupX2 = 415;//390
    private static final double middle_block_pickup_Y2 = -485;
     private static final double right_block_pickupX4 = 405;//380
    private static final double right_block_pickup_Y4 = -230;
    private static final double targeLifttHeight = 5600; // height for the lift 
    private static final double targeLifttHeightdown = 100; // height for the lift
    private static final double scorePositionX1 = 325; // score position X  
    private static final double scorePositionY1 = -550; // score position Y  
     private static final double scorePositionX2 = 295; // score position X  
    private static final double scorePositionY2 = -590; // score position Y  
    private static final double parkPositionX = 1200; // park position X 
    private static final double parkPositionY = 1200; // park position Y  
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
    private final double clawStepUp = 0.20;
    private final double clawStepDown = .20;
    private static final double clawServoDrop = 0; // score position X
    private static final double clawServoPickup = 0.5; // score position X



    @Override
    public void runOpMode() {
        // Initialize robot hardware
        robot.initHardware(hardwareMap);
        robot.bucketServo.setPosition(startBucket); // Set bucketServo to 0.35 initially
        robot.clawRotate.setPosition(clawRotateBlockDrop);

        // Initialize GoBilda Pinpoint
        pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // Set up Pinpoint driver configuration
        pinpointDriver.setOffsets(-84.0, -168.0);
        pinpointDriver.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpointDriver.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpointDriver.resetPosAndIMU();

        // Initialize encoders for arm motors
        robot.liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); 
        robot.liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); 
        robot.liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER); 
        robot.liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER); 

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot.ArmOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.ArmTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    

        waitForStart();
        
        // Start the first set of movements (combined from both versions)
        Thread liftThread1 = new Thread(new LiftTask1());
        Thread movementThread1 = new Thread(new MovementTask1());
        sleep(350);
        liftThread1.start();
        movementThread1.start();

        try {
            liftThread1.join(); 
            movementThread1.join(); 
        } catch (InterruptedException e) {
            telemetry.addData("Error", "Thread interrupted: " + e.getMessage());
            telemetry.update();
        }
/*
        // Start the second set of movements (combined from both versions)
        Thread forwardThread2 = new Thread(new forwardMovement2());
        Thread limitthreadDown2 = new Thread(new limitDown2());
        sleep(350); 
      limitthreadDown2.start();
        forwardThread2.start();

        try {
            forwardThread2.join(); 
            limitthreadDown2.join();
        } catch (InterruptedException e) {
            telemetry.addData("Error", "Thread interrupted: " + e.getMessage());
            telemetry.update();
        }

        // Start the scoring movements (combined from both versions)
        Thread scoreThread3 = new Thread(new ScoreMovement3());
        Thread scoreLift3 = new Thread(new scorelimitLift3());
        sleep(550);
        scoreLift3.start();
        scoreThread3.start();

        try {
            scoreThread3.join(); 
            scoreLift3.join(); 
        } catch (InterruptedException e) {
            telemetry.addData("Error", "Thread interrupted: " + e.getMessage());
            telemetry.update();
        }
        Thread forwardThread4 = new Thread(new forwardMovement4());
        Thread limitthreadDown4 = new Thread(new limitDown4());
        sleep(350); 
        limitthreadDown4.start();
        forwardThread4.start();

        try {
            forwardThread4.join(); 
            limitthreadDown4.join();
        } catch (InterruptedException e) {
            telemetry.addData("Error", "Thread interrupted: " + e.getMessage());
            telemetry.update();
        }
         Thread forwardThread6 = new Thread(new forwardMovement6());
        Thread limitthreadDown6 = new Thread(new limitDown6());
        sleep(350); 
        limitthreadDown6.start();
        forwardThread6.start();

        try {
            forwardThread6.join(); 
            limitthreadDown6.join();
        } catch (InterruptedException e) {
            telemetry.addData("Error", "Thread interrupted: " + e.getMessage());
            telemetry.update();
        }
        
        sleep(550);
       
        
        Thread forwardThread5 = new Thread(new forwardMovement5());
        sleep(350); 
        forwardThread5.start();

        try {
            forwardThread5.join(); 
        } catch (InterruptedException e) {
            telemetry.addData("Error", "Thread interrupted: " + e.getMessage());
            telemetry.update();
        }
        */
}
    // Task to handle bringing the lift up (improved from both versions)
    private class LiftTask1 implements Runnable {
        @Override
        public void run() {
            robot.ArmOne.setPower(-0.85);  // Optimized power from version 2
            robot.ArmTwo.setPower(-0.85);
            sleep(400);  // Optimized sleep from version 2
            robot.ArmOne.setPower(-0.2);
            robot.ArmTwo.setPower(0.2);
            robot.bucketServo.setPosition(neutralBucket);
            robot.clawRotate.setPosition(clawRotateBlockRight);



            robot.liftLeft.setPower(-0.4);
            robot.liftRight.setPower(-0.4);

            while (opModeIsActive() && (Math.abs(robot.liftLeft.getCurrentPosition()) < targeLifttHeight)) {
             //   telemetry.addData("lift left Encoder", robot.liftLeft.getCurrentPosition());
               // telemetry.addData("lift right Encoder", robot.liftRight.getCurrentPosition());
                //telemetry.addData("Target Height", targeLifttHeight);
                //telemetry.update();
            }

            robot.liftLeft.setPower(0.05);
            robot.liftRight.setPower(0.05);
            sleep(300);

            robot.bucketServo.setPosition(dropBucket); 
            sleep(1000);
           //robot.bucketServo.setPosition(neutralBucket);
        }
    }

    // Task to handle bringing the lift down (improved from both versions)
    private class limitDown2 implements Runnable {
        @Override
        public void run() {
            robot.liftLeft.setPower(0.4);
            robot.liftRight.setPower(0.4);

            while (opModeIsActive() && (Math.abs(robot.liftLeft.getCurrentPosition()) > targeLifttHeightdown)) {
                telemetry.addData("lift left Encoder", robot.liftLeft.getCurrentPosition());
                telemetry.addData("lift right Encoder", robot.liftRight.getCurrentPosition());
                telemetry.addData("Target Height", targeLifttHeight);
                telemetry.update();
            }
            robot.liftLeft.setPower(0.0);
            robot.liftRight.setPower(0.0);
        }
    }

    // Task to handle bringing the arm into scoring position (from version 2)
    private class scorelimitLift3 implements Runnable {
        @Override
        public void run() {
            robot.liftLeft.setPower(0.4);
            robot.liftRight.setPower(0.4);

            while (opModeIsActive() && (Math.abs(robot.liftLeft.getCurrentPosition()) < targeLifttHeight)) {
                telemetry.addData("lift left Encoder", robot.liftLeft.getCurrentPosition());
                telemetry.addData("lift right Encoder", robot.liftRight.getCurrentPosition());
                telemetry.addData("Target Height", targeLifttHeight);
                telemetry.update();
            }

            robot.liftLeft.setPower(0.05);
            robot.liftRight.setPower(0.05);
            sleep(100);

            robot.bucketServo.setPosition(dropBucket); 
            sleep(1000);
            robot.bucketServo.setPosition(neutralBucket); 
        }
    }
    
     private class limitDown4 implements Runnable {
        @Override
        public void run() {
            robot.liftLeft.setPower(-0.4);
            robot.liftRight.setPower(-0.4);

            while (opModeIsActive() && (Math.abs(robot.liftLeft.getCurrentPosition()) > targeLifttHeightdown)) {
                telemetry.addData("lift left Encoder", robot.liftLeft.getCurrentPosition());
                telemetry.addData("lift right Encoder", robot.liftRight.getCurrentPosition());
                telemetry.addData("Target Height", targeLifttHeight);
                telemetry.update();
            }
            robot.liftLeft.setPower(0.0);
            robot.liftRight.setPower(0.0);
        }
    }
     private class limitDown6 implements Runnable {
        @Override
        public void run() {
            robot.liftLeft.setPower(0.4);
            robot.liftRight.setPower(0.4);

            while (opModeIsActive() && (Math.abs(robot.liftLeft.getCurrentPosition()) < targeLifttHeight)) {
                telemetry.addData("lift left Encoder", robot.liftLeft.getCurrentPosition());
                telemetry.addData("lift right Encoder", robot.liftRight.getCurrentPosition());
                telemetry.addData("Target Height", targeLifttHeight);
                telemetry.update();
            }

            robot.liftLeft.setPower(0.0);
            robot.liftRight.setPower(0.0);
            sleep(100);

            robot.bucketServo.setPosition(dropBucket); 
            sleep(1000);
            robot.bucketServo.setPosition(neutralBucket); 
        }
    }

    // Task to handle movement based on odometry feedback (improved from both versions)
    private class MovementTask1 implements Runnable {
        @Override
        public void run() {
            while (!movementComplete1 && opModeIsActive()) {
                pinpointDriver.update();
                Pose2D pose = pinpointDriver.getPosition();
                double x = pose.getX(DistanceUnit.MM);
                double y = pose.getY(DistanceUnit.MM);
 // 125
                if (x < TARGET_POSITIONX1) {
                    robot.setDriverMotorPower(0.4, 0.4, 0.4, 0.4);
                } else if (y < TARGET_POSITIONY1) {
                    // -515
                    robot.setDriverMotorPower(0.5, -0.5, -0.5, 0.5);
                } else {
                    robot.setDriverMotorPower(0, 0, 0, 0); 
                    movementComplete1 = true;
                }

                telemetry.addData("X (mm)", x);
                telemetry.addData("Y (mm)", y);
                telemetry.update();
            }
        }
    }

    // Task to move the robot into the correct position to score (improved from both versions)
    private class forwardMovement2 implements Runnable {
        @Override
        public void run() {
            while (!movementComplete2 && opModeIsActive()) {
                pinpointDriver.update(); 
                Pose2D pose = pinpointDriver.getPosition();
                double x = pose.getX(DistanceUnit.MM);
                double y =  -1 * pose.getY(DistanceUnit.MM);

                telemetry.addData("Forwards Movement X (mm)", x);
                telemetry.addData("Forwards Movement Y (mm)", y);
                telemetry.addData("Thread Status", "Moving forward");
                telemetry.update();

                if (x < middle_block_pickupX2) {
                    robot.setDriverMotorPower(0.4, 0.4, 0.4, 0.4);
                } else if (y < middle_block_pickup_Y2) {
                    robot.setDriverMotorPower(-0.5, 0.5, 0.5, -0.5);
                    robot.clawServo.setPosition(clawServoPickup); // Assuming this opens the claw
                } else {
                    robot.setDriverMotorPower(0, 0, 0, 0); 
                    movementComplete2 = true;
                    sleep(300);
                    robot.clawServo.setPosition(clawOpen);

                   // arm going down
                    while (opModeIsActive() && !robot.touchgrab.isPressed()) {
                        robot.ArmOne.setPower(-.4); 
                        robot.ArmTwo.setPower(-.4);
                    }

                    robot.ArmOne.setPower(0); 
                    robot.ArmTwo.setPower(0);
                    sleep(500);
                    robot.clawServo.setPosition(clawClose); // Assuming this closes the claw
                    sleep(500);

                    while (opModeIsActive() && !robot.touchdrop.isPressed()) {
                        robot.ArmOne.setPower(.35); 
                        robot.ArmTwo.setPower(.35);
                        robot.clawRotate.setPosition(clawRotateBlockDrop);
                    }
                    robot.ArmOne.setPower(0); 
                    robot.ArmTwo.setPower(0);
                    
                     robot.clawServo.setPosition(clawServoDrop); // Assuming this opens the claw
                    sleep(1000);
                        robot.ArmOne.setPower(-.3); 
                        robot.ArmTwo.setPower(-.3); 
                        sleep(300);
                        robot.ArmOne.setPower(0); 
                        robot.ArmTwo.setPower(0); 
                }
            }
        }
        }
        

    // Task to handle scoring movements (from version 2)
    private class ScoreMovement3 implements Runnable {
        @Override
        public void run() {
            while (!movementComplete3 && opModeIsActive()) {
                pinpointDriver.update(); 
                Pose2D pose = pinpointDriver.getPosition();
                double x = pose.getX(DistanceUnit.MM);
                double y = pose.getY(DistanceUnit.MM);

                if (x > scorePositionX1) {
                    robot.setDriverMotorPower(-0.4, -0.4, -0.4, -0.4); 
                } else if (y > scorePositionY1) {
                    robot.setDriverMotorPower(0.5, -0.5, -0.5, 0.5); 
                } else {
                    robot.setDriverMotorPower(0, 0, 0, 0); 
                    movementComplete3 = true;
                }

                telemetry.addData("X (mm)", x);
                telemetry.addData("Y (mm)", y);
                telemetry.update();
            }
        }
    }
    // Task to move the robot into the correct position to score (improved from both versions)
    private class forwardMovement4 implements Runnable {
        @Override
        public void run() {
            while (!movementComplete4 && opModeIsActive()) {
                pinpointDriver.update(); 
                Pose2D pose = pinpointDriver.getPosition();
                double x = pose.getX(DistanceUnit.MM);
                double y = pose.getY(DistanceUnit.MM);

                telemetry.addData("Forwards Movement X (mm)", x);
                telemetry.addData("Forwards Movement Y (mm)", y);
                telemetry.addData("Thread Status", "Moving forward");
                telemetry.update();

                if (x < right_block_pickupX4) {
                    robot.setDriverMotorPower(0.4, 0.4, 0.4, 0.4); 
                } else if (y < right_block_pickup_Y4) {
                    robot.setDriverMotorPower(-0.5, 0.5, 0.5, -0.5); 
                } else {
                   robot.setDriverMotorPower(0, 0, 0, 0); 
                    movementComplete4 = true;
                    sleep(300);
                    robot.clawServo.setPosition(clawOpen);

                   // This part is taken from the first version, with optimized values
                    while (opModeIsActive() && !robot.touchgrab.isPressed()) {
                        robot.ArmOne.setPower(-.35); 
                        robot.ArmTwo.setPower(-.35);
                    }
                    robot.ArmOne.setPower(0); 
                    robot.ArmTwo.setPower(0);
                    sleep(1000); 
                    robot.clawServo.setPosition(clawClose); // Assuming this closes the claw
                    sleep(500);

                    while (opModeIsActive() && !robot.touchdrop.isPressed()) {
                        robot.ArmOne.setPower(.35); 
                        robot.ArmTwo.setPower(.35);
                    }
                    robot.ArmOne.setPower(0); 
                    robot.ArmTwo.setPower(0);
                    
                     robot.clawServo.setPosition(clawOpen); // Assuming this opens the claw
                    sleep(1000);
                        robot.ArmOne.setPower(-.3); 
                        robot.ArmTwo.setPower(-.3); 
                        sleep(300);
                        robot.ArmOne.setPower(0); 
                        robot.ArmTwo.setPower(0); 
                }
            }
        }
        }
        // im lazy so movemnt 6 is just movement 3 but differnet
         private class forwardMovement6 implements Runnable {
        @Override
        public void run() {
            while (!movementComplete6 && opModeIsActive()) {
                pinpointDriver.update(); 
                Pose2D pose = pinpointDriver.getPosition();
                double x = pose.getX(DistanceUnit.MM);
                double y = pose.getY(DistanceUnit.MM);

                if (x > scorePositionX2) {
                    robot.setDriverMotorPower(-0.4, -0.4, -0.4, -0.4); 
                } else if (y > scorePositionY2) {
                    robot.setDriverMotorPower(0.5, -0.5, -0.5, 0.5); 
                } else {
                    robot.setDriverMotorPower(0, 0, 0, 0); 
                    movementComplete6 = true;
                }

                telemetry.addData("X (mm)", x);
                telemetry.addData("Y (mm)", y);
                telemetry.update();
            }
        }
    }
        private class forwardMovement5 implements Runnable {
        @Override
        public void run() {
            while (!movementComplete5 && opModeIsActive()) {
                pinpointDriver.update(); 
                Pose2D pose = pinpointDriver.getPosition();
                double x = pose.getX(DistanceUnit.MM);
                double y = pose.getY(DistanceUnit.MM);

                telemetry.addData("Forwards Movement X (mm)", x);
                telemetry.addData("Forwards Movement Y (mm)", y);
                telemetry.addData("Thread Status", "Moving forward");
                telemetry.update();

                if (x < parkPositionX) {
                    robot.setDriverMotorPower(0.4, 0.4, 0.4, 0.4); 
                } else if (y < parkPositionY) {
                    robot.setDriverMotorPower(-0.5, 0.5, 0.5, -0.5); 
                } else {
                   robot.setDriverMotorPower(0, 0, 0, 0); 
                    movementComplete5 = true;
                    sleep(300);
                    robot.clawServo.setPosition(clawOpen);

                   // This part is taken from the first version, with optimized values
                }
        }
        }


    // Helper method to sleep without exception interruptions
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

}