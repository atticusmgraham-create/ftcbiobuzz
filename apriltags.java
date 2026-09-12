package org.firstinspires.ftc.teamcode;

import com.qualifiers.robotcore.eventloop.opmode.LinearOpMode;
import com.qualifiers.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import android.util.Size;
import java.util.List;

@TeleOp(name = "BIOBUZZ: Native AprilTag C270", group = "Vision")
public class BioBuzzVisionOpMode extends LinearOpMode {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        // 1. Initialize the built-in AprilTag processor 
        aprilTag = new AprilTagProcessor.Builder()
            .setDrawAxes(true)
            .setDrawCubeProjection(true)
            .setDrawTagOutline(true)
            .build();

        // 2. Build the Vision Portal using your Logitech C270 webcam
        visionPortal = new VisionPortal.Builder()
            .setCamera(hardwareMap.get(org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName.class, "Webcam 1"))
            .setCameraResolution(new Size(640, 480)) // Standard resolution for C270
            .addProcessor(aprilTag)
            .build();

        telemetry.addData("Status", "Initialized. Camera Ready.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            telemetry.addData("# Tags Detected", currentDetections.size());

            // 3. Step through detections and output telemetry tracking variables
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    // Range gives you straight line distance in inches
                    telemetry.addLine(String.format("Range: %6.1f inches", detection.ftcPose.range));
                    // Bearing gives you horizontal angle in degrees
                    telemetry.addLine(String.format("Bearing: %6.1f deg", detection.ftcPose.bearing));
                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown Tag", detection.id));
                }
            }
            
            telemetry.update();
            sleep(20); 
        }

        visionPortal.close();
    }
}
