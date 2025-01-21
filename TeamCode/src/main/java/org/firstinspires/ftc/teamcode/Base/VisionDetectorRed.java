// package org.firstinspires.ftc.teamcode;
// import org.firstinspires.ftc.teamcode.Base.AutoRobotStruct;
// import org.firstinspires.ftc.vision.tfod.TfodProcessor;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import org.firstinspires.ftc.vision.tfod.TfodProcessor;
// import org.firstinspires.ftc.vision.VisionPortal;
// import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
// import com.qualcomm.robotcore.hardware.ColorSensor;
// import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
// 
// import java.util.List;
// 
// public class VisionDetectorRed extends AutoRobotStruct {
//     private TfodProcessor tfod;
//     private VisionPortal visionPortal;
//     public boolean detectionActive = true;
// 
//     public VisionDetectorRed(LinearOpMode opMode) {
//         initTfod(opMode);
//     }
// 
//     public void initTfod(LinearOpMode opMode) {
//         tfod = new TfodProcessor.Builder()
//                 .setModelFileName("RedTraining.tflite")
//                 .build();
//         VisionPortal.Builder builder = new VisionPortal.Builder();
//         builder.setCamera(opMode.hardwareMap.get(WebcamName.class, "Webcam1"));
//         builder.addProcessor(tfod);
//         visionPortal = builder.build();
//     }
// 
//     public void startDetection() {
//         detectionActive = true;
//         visionPortal.resumeStreaming();
//     }
// 
//     public void stopDetection() {
//         detectionActive = false;
//         visionPortal.stopStreaming();
//     }
// 
//     public List<Recognition> getRecognitions() {
//         return tfod.getRecognitions();
//     }
// 
//     public void telemetryTfod(LinearOpMode opMode) {
//         if (!detectionActive) {
//             return;
//         }
// 
//         List<Recognition> currentRecognitions = getRecognitions();
//         opMode.telemetry.addData("# Objects Detected", currentRecognitions.size());
// 
//         for (Recognition recognition : currentRecognitions) {
//             double x = (recognition.getLeft() + recognition.getRight()) / 2;
//             double y = (recognition.getTop() + recognition.getBottom()) / 2;
//             opMode.telemetry.addData("Object Position (X, Y)", "%.2f, %.2f", x, y);
//         }
// 
//         opMode.telemetry.update();
//     }
// }
// 
// 
// 
// 