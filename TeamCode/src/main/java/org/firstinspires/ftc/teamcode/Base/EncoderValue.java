package org.firstinspires.ftc.teamcode.Base;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
public class EncoderValue {
    public DcMotor liftLeft, liftRight;

    public EncoderValue() {
        liftLeftPosition = Math.abs(liftLeft.getCurrentPosition());
        liftRightPosition = Math.abs(liftRight.getCurrentPosition());
    }

    public double liftLeftPosition;
    public double liftRightPosition;
}



