package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.GyroSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name = "GyroSensorTest")
public class GyroSensorTest extends LinearOpMode {

    GyroSensor gyroSensor;
    IntegratingGyroscope gyro;
    @Override
    public void runOpMode() {
        gyroSensor = hardwareMap.get(GyroSensor.class, "GyroSenor");
        // Calibrate gyro (if necessary)
        gyro = (IntegratingGyroscope)gyroSensor;
        gyroSensor.calibrate();
        while (gyroSensor.isCalibrating()) {
            telemetry.addData("Status", "Calibrating gyro...");
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            double currentAngle = gyroSensor.getHeading();
            float otherAngle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            telemetry.addData("Current Angle:", currentAngle);
            telemetry.addData("Other Angle: ", otherAngle);
            telemetry.update();
            
            

            // Add your autonomous actions here, using currentAngle as needed
        }
    }
}
