package org.firstinspires.ftc.mmcenterstage;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class MM_AprilTags {
    private final LinearOpMode opMode;

    public AprilTagProcessor aprilTagProcessor;
    public VisionPortal visionPortal;

    public MM_AprilTags(LinearOpMode opMode) {
        this.opMode = opMode;

        initAprilTag();
    }


    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        // Create the AprilTag processor.
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)

                // ... these parameters are fx, fy, cx, cy.

                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();


        builder.setCamera(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"));

        builder.setCameraResolution(new Size(1920, 1080));

        // Set and enable the processor.
        builder.addProcessor(aprilTagProcessor);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Wait for the DS start button to be touched.
        opMode.telemetry.addData("DS preview on/off\n" +
                "Connected Hardware\n" +
                "\n" +
                "\n" +
                "Check for Updates\n" +
                "Last check: Sat Oct 07 2023\n" +
                "\n" +
                "\n" +
                "No Hardware Detected", "3 dots, Camera Stream");
        opMode.telemetry.addData(">", "Touch Play to start OpMode");
        opMode.telemetry.update();

        opMode.sleep(1000);
    }   // end method initAprilTag()
}