package org.firstinspires.ftc.mmcenterstage;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Config
public class MM_Drivetrain {
    private final MM_OpMode opMode;
    private final ElapsedTime timer = new ElapsedTime();

    private DcMotorEx flMotor = null;
    private DcMotorEx frMotor = null;
    private DcMotorEx blMotor = null;
    private DcMotorEx brMotor = null;
    private IMU imu;
    double heading;

    public static double MAX_DRIVE_POWER = .5;
    public static double MAX_TURN_POWER = .5;
    public static double APRIL_TAG_THRESHOLD = 1;
    public static double DRIVE_P_COEFF = .0166;
    public static double STRAFE_P_COEFF = .05;
    public static double TURN_P_COEFF = .016;
    public static double MIN_DRIVE_POWER = .28;
    public static double MAX_DETECT_ATTEMPTS = 150;
    public static  double MIN_TURN_POWER = .15;
    public final double WHEEL_DIAMETER = 4;
    public final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    public final double TICKS_PER_REVELUTION = 537.7; // for drivetrain only(5202-0002-0027 "753.2" TPR), change for the real robot ("537.7" TPR)
    public final double TICKS_PER_INCH = TICKS_PER_REVELUTION / WHEEL_CIRCUMFERENCE;
    public static double inchesToDrive = 48;
    public static int TURN_THRESHOLD = 2;

    public MM_VisionPortal visionPortal;

    boolean isSlow = false;

    private double flPower = 0;
    private double frPower = 0;
    private double blPower = 0;
    private double brPower = 0;

    int detectAttemptCount = 0;
    double errorY = 0;
    double errorX = 0;

    public MM_Drivetrain(MM_OpMode opMode) {
        this.opMode = opMode;
        init();
    }

    public void driveWithSticks() {
        double drivePower = -opMode.gamepad1.left_stick_y;
        double strafePower = opMode.gamepad1.left_stick_x;
        double rotatePower = opMode.gamepad1.right_stick_x;

        flPower = drivePower + strafePower + rotatePower;
        frPower = drivePower - strafePower - rotatePower;
        blPower = drivePower - strafePower + rotatePower;
        brPower = drivePower + strafePower - rotatePower;

        if (MM_TeleOp.currentGamepad1.a && !MM_TeleOp.previousGamepad1.a && !opMode.gamepad1.start) {
            isSlow = !isSlow;
        }

        normalize(.85);

        if (isSlow) {
            flPower *= 0.5;
            frPower *= 0.5;
            blPower *= 0.5;
            brPower *= 0.5;
        }

        flMotor.setPower(flPower);
        frMotor.setPower(frPower);
        blMotor.setPower(blPower);
        brMotor.setPower(brPower);
    }

    public void driveToAprilTag(int tagToFind, double targetX, double targetY) {
        flMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        blMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        brMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        flMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        blMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        brMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        boolean keepGoing = true;
        detectAttemptCount = 0;
        double drivePower = 0;
        double strafePower = 0;
        AprilTagDetection tagId = null;

        timer.reset();
        while (opMode.opModeIsActive() && keepGoing) {
            tagId = getAprilTagId(tagToFind);
            getTfodId();

            if (tagId != null) {
                errorY = -getErrorY(targetY, tagId);
                errorX = -getErrorX(targetX, tagId);
                detectAttemptCount = 0;

                if(opMode.gamepad2.left_stick_x < .3) {
                    drivePower = errorY * DRIVE_P_COEFF;
                    strafePower = errorX * STRAFE_P_COEFF;
                } else {
                    drivePower = errorY * DRIVE_P_COEFF * MAX_DRIVE_POWER;
                    strafePower = errorX * STRAFE_P_COEFF * MAX_DRIVE_POWER;
                }

                flPower = drivePower + strafePower;
                frPower = drivePower - strafePower;
                blPower = drivePower - strafePower;
                brPower = drivePower + strafePower;

                normalize(MAX_DRIVE_POWER);
                normalizeForMin(MIN_DRIVE_POWER);
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
                    driveInches(-37, .6);
                    keepGoing = false;
                }
                opMode.sleep(1);
            }

            opMode.multipleTelemetry.addData("errorY", errorY);
            opMode.multipleTelemetry.addData("errorX", errorX);
            opMode.multipleTelemetry.addData("detect attempts", detectAttemptCount);
            opMode.multipleTelemetry.addData("powers", " drive: %.2f  :)  strafe: %.2f", drivePower, strafePower);
            opMode.multipleTelemetry.addData("aa fl normalize powers", flPower);
            opMode.multipleTelemetry.addData("ab fr normalize powers", frPower);
            opMode.multipleTelemetry.addData("ac bl normalize powers", blPower);
            opMode.multipleTelemetry.addData("ad br normalize powers", brPower);
            opMode.multipleTelemetry.update();
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

    private void normalizeForMin(double minPower) {
        if (flPower < minPower && frPower < minPower && blPower < minPower && brPower < minPower) {
            double rawMaxPower = Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)),
                    Math.max(Math.abs(blPower), Math.abs(brPower)));
            double multiplier = minPower / rawMaxPower;
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

    public void cruiseUnderTruss(){   //DO NOT RENAME; IF RENAMED THIS WILL BECOME A WAR!!!

    }

    public void strafeInches(double inches, double power) {
        int ticks = (int) (TICKS_PER_INCH * (inches * 1.23));

        flMotor.setTargetPosition(ticks + flMotor.getCurrentPosition());
        frMotor.setTargetPosition(-ticks + frMotor.getCurrentPosition());
        blMotor.setTargetPosition(-ticks + blMotor.getCurrentPosition());
        brMotor.setTargetPosition(ticks + brMotor.getCurrentPosition());

        flMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        frMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        blMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        brMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        flMotor.setPower(power);
        frMotor.setPower(power);
        blMotor.setPower(power);
        brMotor.setPower(power);
        while (opMode.opModeIsActive() && (flMotor.isBusy() || brMotor.isBusy())) { }
    }

    private double getErrorY(double targetDistance, AprilTagDetection tagId) {
        return tagId.ftcPose.y - targetDistance;
    }

    private double getErrorX(double targetDistance, AprilTagDetection tagId) {
        return tagId.ftcPose.x - targetDistance;
    }

    public AprilTagDetection getAprilTagId(int id) {
        List<AprilTagDetection> currentDetections = visionPortal.aprilTagProcessor.getDetections();

        for (AprilTagDetection detection : currentDetections) {
            if (opMode.opModeInInit()) {
                opMode.multipleTelemetry.addLine(String.format("XY (ID %d) %6.1f %6.1f  (inch)", detection.id, detection.ftcPose.x, detection.ftcPose.y));
            }
            if (detection.id == id) {
                return detection;
            }
        }
        return null;
    }

    public void getTfodId() {
        List<Recognition> currentRecognitions = visionPortal.tfod.getRecognitions();
        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;

            if (opMode.opModeInInit()) {
                opMode.multipleTelemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                opMode.multipleTelemetry.addData("- Position", "%.0f / %.0f", x, y);
            }
        }   // end for() loop
    }

    public void rotateToAngle(int targetAngle) {
        flMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        blMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        brMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        flMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        blMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        brMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);


        opMode.multipleTelemetry.addData("current angle", heading);
        opMode.multipleTelemetry.update();

        double error = getYawError(targetAngle, heading);

        while (opMode.opModeIsActive() && Math.abs(error) > TURN_THRESHOLD) {
            double power = error * TURN_P_COEFF * MAX_TURN_POWER;

            flPower = -(power);
            frPower = power;
            blPower = -(power);
            brPower = power;

            //

            normalizeForMin(MIN_TURN_POWER);
            normalize(MAX_TURN_POWER);

            flMotor.setPower(flPower);
            frMotor.setPower(frPower);
            blMotor.setPower(blPower);
            brMotor.setPower(brPower);

            heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            error = getYawError(targetAngle, heading);

            opMode.multipleTelemetry.addData("power", power);
            opMode.multipleTelemetry.addData("error", error);
            opMode.multipleTelemetry.addData("Z", heading);
            opMode.multipleTelemetry.update();
        }
        flMotor.setPower(0);
        frMotor.setPower(0);
        blMotor.setPower(0);
        brMotor.setPower(0);
    }

    private double getYawError(int targetAngle, double currentAngle) {
        double error = targetAngle - currentAngle;

        error = (error > 180) ? error - 360 : ((error <= -180) ? error + 360 : error); // a nested ternary to determine error
        return error;
    }

    public int purplePixelLeft(boolean isBlue){
        int propPos = propPositionLeft();

        if (propPos == 0){
            driveInches(-20, 0.5);
            rotateToAngle(45);
            driveInches(-12, 0.5);
            rotateToAngle(0);
            driveInches(10, 0.5);
//            rotateToAngle(0);
//            driveInches(12, 0.5);
            rotateToAngle(90);
            if (isBlue){
                driveToAprilTag(1, 0, 4.5);
            }
        } else if (propPos == 1){
            driveInches(-30, 0.5);
            driveInches(10, 0.5);
            rotateToAngle(90);
            if (isBlue) {
                driveToAprilTag(2, 0, 4.5);
            }
        } else {
            driveInches(-20, 0.5);
            rotateToAngle(-45);
            driveInches(-11, 0.5);
            driveInches(10, 0.5);
            rotateToAngle(90);
            if (isBlue) {
                driveToAprilTag(3, 0, 4.5);
            }
        }
        return propPos;
    }

    public int purplePixelRight(boolean isBlue){
        int propPos = propPositionRight();

        if (propPos == 0){
            driveInches(-20, 0.5);
            rotateToAngle(45);
            driveInches(-12, 0.5);
            driveInches(15, 0.5);
            rotateToAngle(0);
            driveInches(-10, 0.5);
            rotateToAngle(-90);
            if (!isBlue){
                driveToAprilTag(4, 0, 4.5);
            }
        } else if (propPos == 1){
            driveInches(-30, 0.5);
            driveInches(10, 0.5);
            rotateToAngle(-90);
            if (!isBlue) {
                driveToAprilTag(5, 0, 4.5);
            }
        } else {
            driveInches(-20, 0.5);
            rotateToAngle(-45);
            driveInches(-12, 0.5);
            rotateToAngle(0);
            driveInches(10, 0.5);
            rotateToAngle(-90);
            if (!isBlue) {
                driveToAprilTag(6, 0, 4.5);
            }
        }
        return propPos;
    }

    private int propPositionRight(){
        List <Recognition> recognitions = visionPortal.tfod.getRecognitions();

        for (Recognition recognition : recognitions){
            if (recognition.getWidth() < 150 &&  recognition.getHeight() < 180){
                if (recognition.getLeft() > 450){
                    return 2;
                } else {
                    return 1;
                }
            }
        }
        return 0;
    }

    private int propPositionLeft(){
        List <Recognition> recognitions = visionPortal.tfod.getRecognitions();

        for (Recognition recognition : recognitions){
            opMode.multipleTelemetry.addData("Prop", "width: %.2f, height: %.2f", recognition.getWidth(), recognition.getHeight());
            opMode.multipleTelemetry.update();
            if (recognition.getWidth() < 150 &&  recognition.getHeight() < 180){
                if (recognition.getLeft() > 130){
                    return 1;
                } else {
                    return 0;
                }

            }
        }
        return 2;
    }

    public void init() {
        flMotor = opMode.hardwareMap.get(DcMotorEx.class, "flMotor");
        frMotor = opMode.hardwareMap.get(DcMotorEx.class, "frMotor");
        blMotor = opMode.hardwareMap.get(DcMotorEx.class, "blMotor");
        brMotor = opMode.hardwareMap.get(DcMotorEx.class, "brMotor");

        flMotor.setDirection(DcMotorEx.Direction.REVERSE);
        blMotor.setDirection(DcMotorEx.Direction.REVERSE);

        if (!opMode.getClass().getSimpleName().equals("MM_TeleOp") ) {
            initExtraForAutos();
        }
    }
    public void initExtraForAutos() {
        flMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        blMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        brMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        imu = opMode.hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        visionPortal = new MM_VisionPortal(opMode);

//        dashboardTelemetry.addData("detect attempts", detectAttemptCount);
//        dashboardTelemetry.addData("errorX", errorX);
//        dashboardTelemetry.update();
//
    }
}

