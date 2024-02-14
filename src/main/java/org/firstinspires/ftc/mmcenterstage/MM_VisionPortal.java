package org.firstinspires.ftc.mmcenterstage;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

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
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

@Config
public class MM_VisionPortal {
    private final MM_OpMode opMode;

    public AprilTagProcessor aprilTagProcessor;
    public TfodProcessor tfod;
    public VisionPortal visionPortal;

    private static final String TFOD_MODEL_ASSET = "Random.tflite";
    public static int GAIN = 255;
    public static int EXPOSURE = 15;

    private static final String[] LABELS = {
            "", "", "", "", "", "", "", "", "", "", "prop", "prop"
    };

    public MM_VisionPortal(MM_OpMode opMode) {
        this.opMode = opMode;

        initVisionPortal();
    }

    public int propPosition(){
        List <Recognition> recognitions = tfod.getRecognitions();

        for (Recognition recognition : recognitions){
            if (recognition.getLabel().equals("prop") && recognition.getWidth() < 150 &&  recognition.getHeight() < 180){
                if (recognition.getLeft() > 450){
                    return 2;
                } else if (recognition.getLeft() < 130){
                    return 0;
                } else {
                    return 1;
                }
            }
        }
        return Math.abs(MM_OpMode.leftOrRight - 1);
    }


    public double getErrorY(double targetDistance, AprilTagDetection tagId) {
        return targetDistance - tagId.ftcPose.y - 2.7;
    }

    public double getErrorX(double targetDistance, AprilTagDetection tagId) {
        return targetDistance - tagId.ftcPose.x;
    }

    public  double getErrorYaw(double targetDistance, AprilTagDetection tagId){
        return targetDistance - tagId.ftcPose.yaw;
    }

    public AprilTagDetection getAprilTagInfo(int id) {
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();

        for (AprilTagDetection detection : currentDetections) {
            if (opMode.opModeInInit()) {
                opMode.multipleTelemetry.addLine(String.format("XY (ID %d) %6.1f %6.1f  (inch)", detection.id, detection.ftcPose.x, detection.ftcPose.y));
            }
            if (detection.id == id) {
                opMode.multipleTelemetry.addData("detection", detection.ftcPose.y);
                opMode.multipleTelemetry.update();
                return detection;
            }
        }
        return null;
    }


    private void initVisionPortal() {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        tfod = new TfodProcessor.Builder()
                .setModelLabels(LABELS)
                .setModelAssetName(TFOD_MODEL_ASSET)
                .build();

        final CameraStreamProcessor cameraStreamProcessor = new CameraStreamProcessor();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        visionPortal = builder.setCamera(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .addProcessor(tfod)
                .addProcessor(cameraStreamProcessor)
                .build();

        FtcDashboard.getInstance().startCameraStream(cameraStreamProcessor, 0);

        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) { }

//       ExposureControl exposure = visionPortal.getCameraControl(ExposureControl.class);
//       exposure.setMode(ExposureControl.Mode.Manual);
//       exposure.setExposure(EXPOSURE, TimeUnit.MILLISECONDS);
//
//        GainControl gain = visionPortal.getCameraControl(GainControl.class);
//        gain.setGain(GAIN);
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