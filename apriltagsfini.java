package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;
import java.util.List;

@TeleOp(name = "C270 Roll Tracking FTC", group = "Tracking")
public class AprilTagRollTracking extends LinearOpMode {

    @Override
    public void runOpMode() {
        // 1. Initialize the FTC AprilTag Processor
        AprilTagProcessor aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                // Set custom camera intrinsics [fx, fy, cx, cy] from your Python script
                // Note: FTC relies on inches for camera parameters/tag sizing internally
                .setLensIntrinsics(1430.0, 1430.0, 320.0, 240.0)
                // Convert your 0.165 meters tag size into inches (0.165m = 6.496 inches)
                .setTagSize(6.496) 
                .setOutputUnits(DistanceUnit.METER, AngleUnit.DEGREES)
                .build();

        // 2. Initialize the Vision Portal (Configured to 640x480 resolution)
        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // Match your robot config name
                .addProcessor(aprilTagProcessor)
                .setCameraResolution(new org.opencv.core.Size(640, 480))
                .build();

        telemetry.addData("Status", "Initialized. Camera stream active.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Get all current active detections from the processor
            List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
            
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    // Extract Distance (Z axis in meters)
                    double t_z = detection.ftcPose.z;

                    // Extract the 3x3 Rotation Matrix
                    Mat R = detection.rawPose.R; 
                    
                    // Replicate math.atan2(R[1, 0], R[0, 0]) in Java to isolate Roll
                    // R.get(row, col) returns a double array containing the pixel/matrix values
                    double r10 = R.get(1, 0)[0];
                    double r00 = R.get(0, 0)[0];
                    
                    double rollRadians = Math.atan2(r10, r00);
                    double rollDegrees = Math.toDegrees(rollRadians);

                    // Send telemetry update to the Driver Station phone
                    telemetry.addLine(String.format("Tag ID %d", detection.id));
                    telemetry.addData("Distance (Z)", String.format("%.2fm", t_z));
                    telemetry.addData("Roll Angle", String.format("%.1f°", rollDegrees));
                    telemetry.addLine("-----------------------");
                }
            }
            
            telemetry.update();
            sleep(20); // Minor cooling loop pause
        }

        // Clean up resources on exit
        visionPortal.close();
    }
}
