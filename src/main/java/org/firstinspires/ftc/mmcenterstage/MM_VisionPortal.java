package org.firstinspires.ftc.mmcenterstage;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class MM_VisionPortal {
    private final LinearOpMode opMode;

    public AprilTagProcessor aprilTagProcessor;
    public TfodProcessor tfod;
    public VisionPortal visionPortal;

    private static final String TFOD_MODEL_ASSET = "redBall+BlueBall.tflite";

    private static final String[] LABELS = {
            "blueProp", "redProp"
    };

    public MM_VisionPortal(LinearOpMode opMode) {
        this.opMode = opMode;

        initVisionPortal();
    }


    public List getRecognitions() {
         List<Recognition> currentRecognitions = tfod.getRecognitions();
         return currentRecognitions;
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initVisionPortal() {
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

        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();


        final CameraStreamProcessor cameraStreamProcessor = new CameraStreamProcessor();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"));
        //builder.setCameraResolution(new Size(1920, 1080));

        // Set and enable the processor.
        builder.addProcessor(aprilTagProcessor);
        builder.addProcessor(tfod);
        builder.addProcessor(cameraStreamProcessor);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        FtcDashboard.getInstance().startCameraStream(cameraStreamProcessor, 0);

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

        opMode.sleep(2500);
    }

    public static class CameraStreamProcessor implements VisionProcessor, CameraStreamSource {
        private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(frame, b);
            lastFrame.set(b);
            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
            // do nothing
        }

        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
        }
    }
}