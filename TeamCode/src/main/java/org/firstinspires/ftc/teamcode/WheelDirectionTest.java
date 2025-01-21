package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Base.RobotStructure;

@TeleOp(name = "WheelDirectionTest")
public class WheelDirectionTest extends RobotStructure {
      public void loop() {
        // Run motor and servo control logic
       motorControls();
      }
  public void motorControls() {      
    
     if (gamepad1.a) {   // motorFrontRight
            // liftLeft , liftRight
        motorFrontRight.setPower(.5);
     }
      if (gamepad1.b) {   // motorFrontLeft
            // liftLeft , liftRight
        motorFrontLeft.setPower(.5);
     }
      if (gamepad1.x) {   // motorBackLeft
            // liftLeft , liftRight
        motorBackLeft.setPower(.5);
     }
      if (gamepad1.y) {   // motorBackRight
            // liftLeft , liftRight
        motorBackRight.setPower(.5);
     }
            else {
           motorFrontRight.setPower(0);
           motorFrontLeft.setPower(0);
           motorBackLeft.setPower(0);
           motorBackRight.setPower(0);
            }
}
}