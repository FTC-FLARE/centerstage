package org.firstinspires.ftc.mmcenterstage;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Config
public class MM_Drivetrain {
    private final LinearOpMode opMode;
    private final ElapsedTime timer = new ElapsedTime();

    public static double MAX_DRIVE_POWER = .7;
    public static double APRIL_TAG_THRESHOLD = 2;
    public static double DRIVE_P_COEFF = .0166;
    public static double STRAFE_P_COEFF = .05;
    public static double MIN_DRIVE_POWER = .28;
    public static double MAX_DETECT_ATTEMPTS = 150;
    public final double WHEEL_DIAMETER = 4;
    public final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    public final double TICKS_PER_REVELUTION = 753.2; // for drivetrain only(5202-0002-0027), change for the real robot
    public final double TICKS_PER_INCH = TICKS_PER_REVELUTION / WHEEL_CIRCUMFERENCE;
    public static double inchesToDrive = 48;

    private DcMotorEx flMotor = null;
    private DcMotorEx frMotor = null;
    private DcMotorEx blMotor = null;
    private DcMotorEx brMotor = null;

    //public MM_AprilTags aprilTags;

    private final Telemetry dashboardTelemetry;
    boolean isSlow = false;

    private double flPower = 0;
    private double frPower = 0;
    private double blPower = 0;
    private double brPower = 0;

    int detectAttemptCount = 0;
    double errorY = 0;
    double errorX = 0;

    public MM_Drivetrain(LinearOpMode opMode, Telemetry dashboardTelemetry) {
        this.opMode = opMode;
        this.dashboardTelemetry = dashboardTelemetry;
        init();
    }

    public void driveWithSticks() {
        double drivePower = -opMode.gamepad1.left_stick_y;
        double strafePower = opMode.gamepad1.left_stick_x;
        double rotatePower = opMode.gamepad1.right_stick_x;
        opMode.telemetry.addData("test", "push");

        flPower = drivePower + strafePower + rotatePower;
        frPower = drivePower - strafePower - rotatePower;
        blPower = drivePower - strafePower + rotatePower;
        brPower = drivePower + strafePower - rotatePower;

        if (MM_TeleOp.currentGamepad1.a && !MM_TeleOp.previousGamepad1.a && !opMode.gamepad1.start) {
            isSlow = !isSlow;
        }

        normalize(1);

        if (isSlow) {
            flPower *= 0.5;
            frPower *= 0.5;
            blPower *= 0.5;
            brPower *= 0.5;
        }

        flMotor.setPower(flPower * 0.7);
        frMotor.setPower(frPower * 0.7);
        blMotor.setPower(blPower * 0.7);
        brMotor.setPower(brPower * 0.7);


    }

    public void driveToAprilTag() {
        boolean keepGoing = true;
        detectAttemptCount = 0;
        double drivePower = 0;
        double strafePower = 0;
        AprilTagDetection tagId = null;

        timer.reset();
        while (opMode.opModeIsActive() && keepGoing) {
            //tagId = getAprilTagId(2);
            //getTfodId();

            if (tagId != null) {
                errorY = getErrorY(6, tagId);
                errorX = getErrorX(0, tagId);
                detectAttemptCount = 0;

                drivePower = errorY * DRIVE_P_COEFF * MAX_DRIVE_POWER;
                strafePower = errorX * STRAFE_P_COEFF * MAX_DRIVE_POWER;

                flPower = drivePower + strafePower;
                frPower = drivePower - strafePower;
                blPower = drivePower - strafePower;
                brPower = drivePower + strafePower;

                normalize(MAX_DRIVE_POWER);
                normalizeForMin();
//
//                flPower = updateForMinPower(flPower);
//                frPower = updateForMinPower(frPower);
//                blPower = updateForMinPower(blPower);
//                brPower = updateForMinPower(brPower);

//                flPower = Math.max(Math.abs(flPower), MIN_DRIVE_POWER);
//                frPower = Math.max(frPower, MIN_DRIVE_POWER);
//                blPower = Math.max(blPower, MIN_DRIVE_POWER);
//                brPower = Math.max(brPower, MIN_DRIVE_POWER);

                flMotor.setPower(flPower);
                frMotor.setPower(frPower);
                blMotor.setPower(blPower);
                brMotor.setPower(brPower);

                if (Math.abs(errorY) <= APRIL_TAG_THRESHOLD && Math.abs(errorX) <= APRIL_TAG_THRESHOLD) {
                    keepGoing = false;
                }
            } else {
                detectAttemptCount++;
                if (detectAttemptCount >= MAX_DETECT_ATTEMPTS) {
                    keepGoing = false;
                }
                opMode.sleep(1);
            }

            dashboardTelemetry.addData("errorY", errorY);
            dashboardTelemetry.addData("errorX", errorX);
            dashboardTelemetry.addData("detect attempts", detectAttemptCount);
            dashboardTelemetry.addData("powers", " drive: %.2f  :)  strafe: %.2f", drivePower, strafePower);
            dashboardTelemetry.addData("aa fl normalize powers", flPower);
            dashboardTelemetry.addData("ab fr normalize powers", frPower);
            dashboardTelemetry.addData("ac bl normalize powers", blPower);
            dashboardTelemetry.addData("ad br normalize powers", brPower);
            dashboardTelemetry.update();
        }//end while keep going

        flMotor.setPower(0);
        frMotor.setPower(0);
        blMotor.setPower(0);
        brMotor.setPower(0);
    }

    public void driveInches(double inches, double power){
        int ticks = (int) (TICKS_PER_INCH * inches);
        
        flMotor.setTargetPosition(ticks + flMotor.getCurrentPosition());
        frMotor.setTargetPosition(ticks + frMotor.getCurrentPosition());
        blMotor.setTargetPosition(ticks + blMotor.getCurrentPosition());
        brMotor.setTargetPosition(ticks + brMotor.getCurrentPosition());

        flMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        frMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        blMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        brMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        flMotor.setPower(power);
        frMotor.setPower(power);
        blMotor.setPower(power);
        brMotor.setPower(power);
        while(opMode.opModeIsActive() && (flMotor.isBusy() || brMotor.isBusy())){

        }
    }

    private void normalizeForMin() {
        if (flPower < MIN_DRIVE_POWER && frPower < MIN_DRIVE_POWER && blPower < MIN_DRIVE_POWER && brPower < MIN_DRIVE_POWER) {
            double rawMaxPower = Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)),
                    Math.max(Math.abs(blPower), Math.abs(brPower)));
            double multiplier = MIN_DRIVE_POWER / rawMaxPower;
            flPower *= multiplier;
            frPower *= multiplier;
            blPower *= multiplier;
            brPower *= multiplier;
        }
    }
    private double updateForMinPower(double motorPower) {
        if (Math.abs(motorPower) < MIN_DRIVE_POWER) {
            if (motorPower < 0) {
                return -MIN_DRIVE_POWER;
            } else {
                return MIN_DRIVE_POWER;
            }
        }
        return motorPower;
    }

    private void normalize(double upperPowerLimit) {
        double rawMaxPower = Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)),
                Math.max(Math.abs(blPower), Math.abs(brPower)));

        if (rawMaxPower > upperPowerLimit) {
            flPower /= rawMaxPower;
            frPower /= rawMaxPower;
            blPower /= rawMaxPower;
            brPower /= rawMaxPower;
        }

    }

    private double getErrorY(double targetDistance, AprilTagDetection tagId) {
        return tagId.ftcPose.y - targetDistance;
    }

    private double getErrorX(double targetDistance, AprilTagDetection tagId) {
        return tagId.ftcPose.x - targetDistance;
    }

//    public AprilTagDetection getAprilTagId(int id) {
//        List<AprilTagDetection> currentDetections = aprilTags.aprilTagProcessor.getDetections();
//
//        for (AprilTagDetection detection : currentDetections) {
//            if (opMode.opModeInInit()) {
//                dashboardTelemetry.addLine(String.format("XY (ID %d) %6.1f %6.1f  (inch)", detection.id, detection.ftcPose.x, detection.ftcPose.y));
//            }
//            if (detection.id == id) {
//                return detection;
//            }
//        }
//        return null;
//    }

//    public void getTfodId() {
//        List<Recognition> currentRecognitions = aprilTags.tfod.getRecognitions();
//        // Step through the list of recognitions and display info for each one.
//        for (Recognition recognition : currentRecognitions) {
//            double x = (recognition.getLeft() + recognition.getRight()) / 2;
//            double y = (recognition.getTop() + recognition.getBottom()) / 2;
//
//            if (opMode.opModeInInit()) {
//                dashboardTelemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
//                dashboardTelemetry.addData("- Position", "%.0f / %.0f", x, y);
//            }
//        }   // end for() loop
//    }


    public void init() {
        flMotor = opMode.hardwareMap.get(DcMotorEx.class, "flMotor");
        frMotor = opMode.hardwareMap.get(DcMotorEx.class, "frMotor");
        blMotor = opMode.hardwareMap.get(DcMotorEx.class, "blMotor");
        brMotor = opMode.hardwareMap.get(DcMotorEx.class, "brMotor");

        flMotor.setDirection(DcMotorEx.Direction.REVERSE);
        blMotor.setDirection(DcMotorEx.Direction.REVERSE);

        //aprilTags = new MM_AprilTags(opMode);



        dashboardTelemetry.addData("detect attempts", detectAttemptCount);
        dashboardTelemetry.addData("errorX", errorX);
        dashboardTelemetry.update();
    }
}

