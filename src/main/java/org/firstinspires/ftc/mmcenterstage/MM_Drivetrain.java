package org.firstinspires.ftc.mmcenterstage;

import static org.firstinspires.ftc.mmcenterstage.MM_Autos.LEFT;
import static org.firstinspires.ftc.mmcenterstage.MM_Autos.RIGHT;
import static org.firstinspires.ftc.mmcenterstage.MM_Autos.leftOrRight;
import static org.firstinspires.ftc.mmcenterstage.MM_Autos.propPos;
import static org.firstinspires.ftc.mmcenterstage.MM_Autos.tagToFindOnWall;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
public class MM_Drivetrain {
    private final MM_OpMode opMode;

    private DcMotorEx flMotor = null;
    private DcMotorEx frMotor = null;
    private DcMotorEx blMotor = null;
    private DcMotorEx brMotor = null;
    private Servo pixelKicker = null;
    public static AnalogInput sonarLeft = null;
    public static AnalogInput sonarRight = null;
    public MM_VisionPortal visionPortal;
    private IMU imu;

    private final ElapsedTime timer = new ElapsedTime();
    ElapsedTime kickTime = new ElapsedTime();

    public static double MAX_DRIVE_POWER = .5;
    public static double MIN_DRIVE_POWER = .28;
    public static double DRIVE_P_COEFF = .0166;

    public static double MAX_STRAFE_POWER = .6;
    public static double STRAFE_P_COEFF = .02;

    public static double MAX_TURN_POWER = .5;
    public static double MIN_TURN_POWER = .15;
    public static double GYRO_TURN_P_COEFF = .016;
    public static double APRIL_TAG_TURN_P_COEFF = .004;
    public static double HEADING_ERROR_THRESHOLD = 1;

    public static double APRIL_TAG_ERROR_THRESHOLD = 2;
    public static double APRIL_TAG_ERROR_THRESHOLD_YAW = 6;

    public static double DISTANCE_THRESHOLD = .5;

    public static double MAX_DETECT_ATTEMPTS = 500;
    public static double MAX_EXPOSURE = 37;

    private final double WHEEL_DIAMETER = 4;
    private final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    private final double TICKS_PER_REVOLUTION = 537.7; // for drivetrain only(5202-0002-0027 "753.2" TPR), change for the real robot ("537.7" TPR)
    private final double TICKS_PER_INCH = TICKS_PER_REVOLUTION / WHEEL_CIRCUMFERENCE;

    public final double BACKDROP_APRILTAG_DISTANCE = 7.3;

    private boolean isSlow = false;
    private boolean isKicking = false;
    private double heading;
    private int kickCount = 0;
    private boolean isConcealed = true;

    private double flPower = 0;
    private double frPower = 0;
    private double blPower = 0;
    private double brPower = 0;

    private int detectAttemptCount = 0;
    public int currentExposure = 32;

    private double aprilTagErrorX = 0;
    private double aprilTagErrorY = 0;
    private double aprilTagErrorYaw = 0;

    private double exposureIterator = 1;

    private boolean increaseExposure = false;



    public MM_Drivetrain(MM_OpMode opMode) {
        this.opMode = opMode;
        init();

    }

    public double  getDistance(int rightLeft){ //TODO remove getDistance from here and tele-op
        return ((rightLeft == 1? sonarLeft.getVoltage(): sonarRight.getVoltage()) * 87.13491 - 12.0424);
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

        setDrivePowers();
    }

    public void kickPixels(){
        if (MM_TeleOp.currentGamepad1.b && !MM_TeleOp.previousGamepad1.b){
            kickTime.reset();
            isKicking = true;
        }
        if (isKicking) {
            if (kickTime.time() >= .6321 || kickCount == 0) {
                if (kickCount < 6) {
                    pixelKicker.setPosition((isConcealed) ? 0 : 1);
                    isConcealed = !isConcealed;
                    kickCount++;
                    kickTime.reset();
                } else {
                    isKicking = false;
                    kickCount = 0;
                }
            }
        }
    }

    public void driveInches(double inches, double power){
        int ticks = (int) (TICKS_PER_INCH * inches);

        flMotor.setTargetPosition(flMotor.getCurrentPosition() + ticks);
        frMotor.setTargetPosition(frMotor.getCurrentPosition() + ticks);
        blMotor.setTargetPosition(blMotor.getCurrentPosition() + ticks);
        brMotor.setTargetPosition(brMotor.getCurrentPosition() + ticks);

        setDriveMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        setDrivePowers(power);
        while(opMode.opModeIsActive() && (flMotor.isBusy() || brMotor.isBusy())){ }
    }

    public void driveInchesAndLowerSlide(double inches, double power){
        int ticks = (int) (TICKS_PER_INCH * inches);

        flMotor.setTargetPosition(flMotor.getCurrentPosition() + ticks);
        frMotor.setTargetPosition(frMotor.getCurrentPosition() + ticks);
        blMotor.setTargetPosition(blMotor.getCurrentPosition() + ticks);
        brMotor.setTargetPosition(brMotor.getCurrentPosition() + ticks);

        setDriveMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        setDrivePowers(power);
        while(opMode.opModeIsActive() && (flMotor.isBusy() || brMotor.isBusy())){
            opMode.robot.transport.goHome();
        }
    }

    public void strafeInches(double inches, double power) {
        int ticks = (int) (TICKS_PER_INCH * (inches * 1.23)); // multiplying to account for slippage

        flMotor.setTargetPosition(ticks + flMotor.getCurrentPosition());
        frMotor.setTargetPosition(-ticks + frMotor.getCurrentPosition());
        blMotor.setTargetPosition(-ticks + blMotor.getCurrentPosition());
        brMotor.setTargetPosition(ticks + brMotor.getCurrentPosition());

        setDriveMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        setDrivePowers(power);
        while (opMode.opModeIsActive() && (flMotor.isBusy() || frMotor.isBusy())) { }
    }
    public void strafeInchesAndLowerSlide(double inches, double power) {
        int ticks = (int) (TICKS_PER_INCH * (inches * 1.23)); // multiplying to account for slippage

        flMotor.setTargetPosition(ticks + flMotor.getCurrentPosition());
        frMotor.setTargetPosition(-ticks + frMotor.getCurrentPosition());
        blMotor.setTargetPosition(-ticks + blMotor.getCurrentPosition());
        brMotor.setTargetPosition(ticks + brMotor.getCurrentPosition());

        setDriveMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        setDrivePowers(power);
        while (opMode.opModeIsActive() && (flMotor.isBusy() || frMotor.isBusy())) {
            opMode.robot.transport.goHome();
        }
    }

    public void rotateToAngle(int targetAngle) {
        setDriveMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double error = getHeadingError(targetAngle, heading);

        while (opMode.opModeIsActive() && Math.abs(error) > HEADING_ERROR_THRESHOLD) {
            double power = error * GYRO_TURN_P_COEFF * MAX_TURN_POWER;

            flPower = -(power);
            frPower = power;
            blPower = -(power);
            brPower = power;

            normalizeForMin(MIN_TURN_POWER);
            normalize(MAX_TURN_POWER);

            setDrivePowers();

            heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            error = getHeadingError(targetAngle, heading);

            opMode.multipleTelemetry.addData("power", power);
            opMode.multipleTelemetry.addData("error", error);
            opMode.multipleTelemetry.addData("heading", heading);
            opMode.multipleTelemetry.update();
        }
        setDrivePowers(0);
    }

    public boolean driveToAprilTag(int tagToFind, double targetX){
        return driveToAprilTag(tagToFind, targetX, BACKDROP_APRILTAG_DISTANCE, -3);
    }

    public boolean driveToAprilTagNoYaw(int tagToFind, double targetX, double targetY) {
        setDriveMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        ElapsedTime refreshTime = new ElapsedTime();

        boolean iSawIt = false;
        boolean keepGoing = true;
        detectAttemptCount = 0;
        double drivePower = 0;
        double strafePower = 0;
        AprilTagDetection tagInfo = null;

        while (opMode.opModeIsActive() && keepGoing) {
            //opMode.robot.transport.moveBucketToScore();
            opMode.multipleTelemetry.addData("current gain", visionPortal.gain.getGain());
            tagInfo = visionPortal.getAprilTagInfo(tagToFind);

            if (tagInfo != null) {
                iSawIt = true;
                opMode.multipleTelemetry.addLine("seeing tag");
                aprilTagErrorY = visionPortal.getErrorY(targetY, tagInfo);
                aprilTagErrorX = visionPortal.getErrorX(targetX, tagInfo);
                detectAttemptCount = 0;

                drivePower = aprilTagErrorY * DRIVE_P_COEFF * MAX_DRIVE_POWER;
                strafePower = aprilTagErrorX * STRAFE_P_COEFF * MAX_DRIVE_POWER;

                flPower = drivePower + strafePower;
                frPower = drivePower - strafePower;
                blPower = drivePower - strafePower;
                brPower = drivePower + strafePower;

                normalizeForMin(MIN_DRIVE_POWER);
                normalize(MAX_DRIVE_POWER);

                setDrivePowers();

                if (Math.abs(aprilTagErrorY) <= APRIL_TAG_ERROR_THRESHOLD && Math.abs(aprilTagErrorX) <= APRIL_TAG_ERROR_THRESHOLD) {
                    setDrivePowers(0);
                    opMode.multipleTelemetry.addLine("Goal reached.");
                    opMode.multipleTelemetry.update();
                    return true;
                }

            } else { //tag not found
                opMode.multipleTelemetry.addLine("looking for tag");
                detectAttemptCount++;

                opMode.multipleTelemetry.addData("current exposure", currentExposure);
                if(!iSawIt){
                    setDrivePowers(tagToFind == tagToFindOnWall? .15: -.15);
                } else if (Math.abs(aprilTagErrorY) <= APRIL_TAG_ERROR_THRESHOLD && tagToFind < 7) {
                    setDrivePowers(0);
                    opMode.multipleTelemetry.addLine("eh close enough");
                    opMode.multipleTelemetry.update();
                    return true;
                }

                if (detectAttemptCount >= MAX_DETECT_ATTEMPTS) {//TODO if tag not found
                    setDrivePowers(0);
                    opMode.multipleTelemetry.addLine("lost aprilTag");
                    opMode.multipleTelemetry.update();
                    return false;

                    //driveToFindAprilTag(tagToFind);
                }

                refreshTime.reset();
                while(opMode.opModeIsActive() && refreshTime.milliseconds() <= 2 && tagInfo == null){
                    if (currentExposure >= MAX_EXPOSURE){
                        increaseExposure = false;
                    } else if (currentExposure < 2){
                        increaseExposure = true;
                    }

                    visionPortal.exposure.setExposure(increaseExposure ? (long) (currentExposure + exposureIterator) : (long) (currentExposure - exposureIterator), TimeUnit.MILLISECONDS);
                    currentExposure = (int) (increaseExposure ? (currentExposure + exposureIterator) : (currentExposure - exposureIterator));
                    tagInfo = visionPortal.getAprilTagInfo(tagToFind);
                }
            }

            opMode.multipleTelemetry.addData("errorY", aprilTagErrorY);
            opMode.multipleTelemetry.addData("errorX", aprilTagErrorX);
            opMode.multipleTelemetry.addData("error yaw", aprilTagErrorYaw);
            opMode.multipleTelemetry.addData("detect attempts", detectAttemptCount);
            opMode.multipleTelemetry.addData("powers", " drive: %.2f  :)  strafe: %.2f", drivePower, strafePower);
            opMode.multipleTelemetry.addData("aa fl normalize powers", flPower);
            opMode.multipleTelemetry.addData("ab fr normalize powers", frPower);
            opMode.multipleTelemetry.addData("ac bl normalize powers", blPower);
            opMode.multipleTelemetry.addData("ad br normalize powers", brPower);
            opMode.multipleTelemetry.update();
        }//end while keep going

        return false;
    }

    public boolean driveToAprilTag(int tagToFind, double targetX, double targetY, double targetYaw) {
        setDriveMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        ElapsedTime refreshTime = new ElapsedTime();

        boolean iSawIt = false;
        boolean keepGoing = true;
        detectAttemptCount = 0;
        double drivePower = 0;
        double strafePower = 0;
        double rotatePower = 0;
        AprilTagDetection tagInfo = null;

        while (opMode.opModeIsActive() && keepGoing) {
            opMode.robot.transport.moveBucketToScore();
            opMode.multipleTelemetry.addData("current gain", visionPortal.gain.getGain());
            tagInfo = visionPortal.getAprilTagInfo(tagToFind);

            if (tagInfo != null) {
                iSawIt = true;
                opMode.multipleTelemetry.addLine("seeing tag");
                aprilTagErrorY = visionPortal.getErrorY(targetY, tagInfo);
                aprilTagErrorX = visionPortal.getErrorX(targetX, tagInfo);
                aprilTagErrorYaw = visionPortal.getErrorYaw(targetYaw, tagInfo);
                detectAttemptCount = 0;

                drivePower = aprilTagErrorY * DRIVE_P_COEFF * MAX_DRIVE_POWER;
                strafePower = aprilTagErrorX * STRAFE_P_COEFF * MAX_DRIVE_POWER;
                rotatePower = aprilTagErrorYaw * APRIL_TAG_TURN_P_COEFF * MAX_DRIVE_POWER;

                flPower = drivePower + strafePower + rotatePower;
                frPower = drivePower - strafePower - rotatePower;
                blPower = drivePower - strafePower + rotatePower;
                brPower = drivePower + strafePower - rotatePower;

                normalizeForMin(MIN_DRIVE_POWER);
                normalize(MAX_DRIVE_POWER);

                setDrivePowers();

                if (Math.abs(aprilTagErrorY) <= APRIL_TAG_ERROR_THRESHOLD && Math.abs(aprilTagErrorX) <= APRIL_TAG_ERROR_THRESHOLD && Math.abs(aprilTagErrorYaw) <= APRIL_TAG_ERROR_THRESHOLD_YAW) {
                    setDrivePowers(0);
                    opMode.multipleTelemetry.addLine("Goal reached.");
                    opMode.multipleTelemetry.update();
                    return true;
                }

            } else { //tag not found
                opMode.multipleTelemetry.addLine("looking for tag");
                detectAttemptCount++;

                opMode.multipleTelemetry.addData("current exposure", currentExposure);
                if(!iSawIt){
                    setDrivePowers(tagToFind == tagToFindOnWall? .15: -.15);
                } else if (Math.abs(aprilTagErrorY) <= APRIL_TAG_ERROR_THRESHOLD && tagToFind < 7) {
                    setDrivePowers(0);
                    opMode.multipleTelemetry.addLine("eh close enough");
                    opMode.multipleTelemetry.update();
                    return true;
                }

                if (detectAttemptCount >= MAX_DETECT_ATTEMPTS) {//TODO if tag not found
                    setDrivePowers(0);
                    opMode.multipleTelemetry.addLine("lost aprilTag");
                    opMode.multipleTelemetry.update();
                    return false;

                    //driveToFindAprilTag(tagToFind);
                }

                refreshTime.reset();
                while(opMode.opModeIsActive() && refreshTime.milliseconds() <= 2 && tagInfo == null){
                    if (currentExposure >= MAX_EXPOSURE){
                        increaseExposure = false;
                    } else if (currentExposure < 2){
                        increaseExposure = true;
                    }

                    visionPortal.exposure.setExposure(increaseExposure ? (long) (currentExposure + exposureIterator) : (long) (currentExposure - exposureIterator), TimeUnit.MILLISECONDS);
                    currentExposure = (int) (increaseExposure ? (currentExposure + exposureIterator) : (currentExposure - exposureIterator));
                    tagInfo = visionPortal.getAprilTagInfo(tagToFind);
                }
            }

            opMode.multipleTelemetry.addData("errorY", aprilTagErrorY);
            opMode.multipleTelemetry.addData("errorX", aprilTagErrorX);
            opMode.multipleTelemetry.addData("error yaw", aprilTagErrorYaw);
            opMode.multipleTelemetry.addData("detect attempts", detectAttemptCount);
            opMode.multipleTelemetry.addData("powers", " drive: %.2f  :)  strafe: %.2f", drivePower, strafePower);
            opMode.multipleTelemetry.addData("aa fl normalize powers", flPower);
            opMode.multipleTelemetry.addData("ab fr normalize powers", frPower);
            opMode.multipleTelemetry.addData("ac bl normalize powers", blPower);
            opMode.multipleTelemetry.addData("ad br normalize powers", brPower);
            opMode.multipleTelemetry.update();
        }//end while keep going

        return false;
    }


    public void driveToFindAprilTag(int tagId){
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setDrivePowers(.45);
        boolean foundTag = false;
        while (opMode.opModeIsActive() && !foundTag ) {
            List<AprilTagDetection> currentDetections = visionPortal.aprilTagProcessor.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if (opMode.opModeInInit()) {
                    opMode.multipleTelemetry.addLine(String.format("XY (ID %d) %6.1f %6.1f  (inch)", detection.id, detection.ftcPose.x, detection.ftcPose.y));
                }
                if (detection.id == tagId) {
                    foundTag = true;
                    break;
                }
            }
        }
        setDrivePowers(0);
    }
    public void park (int propPos) {
        if (propPos == 0) {
            strafeInchesAndLowerSlide(-31, .3);
        } else if (propPos == 1) {
            strafeInchesAndLowerSlide(-24, .3);
        } else {
            strafeInchesAndLowerSlide(31, .3);
        }
        driveInches(-12, .3);
    }

    public void cruiseUnderTruss(){   //DO NOT RENAME; IF RENAMED THIS WILL BECOME A WAR!!!
    // TODO finish cruise under truss
        driveToAprilTagNoYaw(tagToFindOnWall, -7 * leftOrRight, 25.3);
        rotateToAngle(-90 * leftOrRight);
        strafeToDistance(leftOrRight, 4.4);
        rotateToAngle(-90 * leftOrRight);
        driveInches(28, .4);
        driveToAprilTagNoYaw(tagToFindOnWall, -18 * leftOrRight, 75.5);
//        strafeInches(24, .6);
    }

    public void strafeToDistance (int rightLeft, double target){
        double power = 0;
        double error = ((target - getDistance(rightLeft)) * rightLeft);

        while (Math.abs(error) >= DISTANCE_THRESHOLD && opMode.opModeIsActive()){
            power = error * STRAFE_P_COEFF * MAX_STRAFE_POWER;

            flPower = power;
            frPower = -power;
            blPower = -power;
            brPower = power;

            normalizeForMin(.08);
            normalize(MAX_STRAFE_POWER);

            setDrivePowers();

            error = ((target - getDistance(rightLeft)) * rightLeft);

            opMode.multipleTelemetry.addData("distance error", error);
            opMode.multipleTelemetry.update();
        }

        setDrivePowers(0);

    }

    public void strafeToFIndAprilTag(int tagId, int rightLeft, double power){
        AprilTagDetection tagInfo = visionPortal.getAprilTagInfo(tagId);

        while (opMode.opModeIsActive() && tagInfo == null){

            if (currentExposure >= MAX_EXPOSURE){
                increaseExposure = false;
            } else if (currentExposure < 2){
                increaseExposure = true;
            }

            visionPortal.exposure.setExposure(increaseExposure ? (long) (currentExposure + exposureIterator) : (long) (currentExposure - exposureIterator), TimeUnit.MILLISECONDS);
            currentExposure = (int) (increaseExposure ? (currentExposure + exposureIterator) : (currentExposure - exposureIterator));
            tagInfo = visionPortal.getAprilTagInfo(tagId);

            flPower = power;
            frPower = -power;
            blPower = -power;
            brPower = power;

            setDrivePowers();

            if (getDistance(rightLeft) >=  30.5){
                setDrivePowers(0);
                break;
            }

        }
    }

    public int purplePixel(){
        propPos = visionPortal.propPosition();

        if ((propPos == 0 && leftOrRight == LEFT) || (propPos == 2 && leftOrRight == RIGHT)){  // away from truss
            strafeInches(-8.5 * leftOrRight, .7);
            driveInches(-27, 0.6);
            driveInches(9, .7);
        } else if (propPos == 1){  // center
            driveInches(-32, 0.6);
            driveInches(8, 0.7);
        } else {  // by truss
            driveInches(-20, 0.6);
            rotateToAngle(45 * leftOrRight);
            driveInches(-11, 0.5);
            driveInches(10, 0.7);
        }
        return propPos;
    }



    private void setDriveMode(DcMotor.RunMode runToPosition) {
        flMotor.setMode(runToPosition);
        frMotor.setMode(runToPosition);
        blMotor.setMode(runToPosition);
        brMotor.setMode(runToPosition);
    }

    private void setDrivePowers(double power) {
        flMotor.setPower(power);
        frMotor.setPower(power);
        blMotor.setPower(power);
        brMotor.setPower(power);
    }

    private void setDrivePowers() {
        flMotor.setPower(flPower);
        frMotor.setPower(frPower);
        blMotor.setPower(blPower);
        brMotor.setPower(brPower);
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

    private double getHeadingError(int targetAngle, double currentAngle) {
        double error = targetAngle - currentAngle;

        error = (error > 180) ? error - 360 : ((error <= -180) ? error + 360 : error); // a nested ternary to determine error
        return error;
    }

    public void init() {
        flMotor = opMode.hardwareMap.get(DcMotorEx.class, "flMotor");
        frMotor = opMode.hardwareMap.get(DcMotorEx.class, "frMotor");
        blMotor = opMode.hardwareMap.get(DcMotorEx.class, "blMotor");
        brMotor = opMode.hardwareMap.get(DcMotorEx.class, "brMotor");

        flMotor.setDirection(DcMotorEx.Direction.REVERSE);
        blMotor.setDirection(DcMotorEx.Direction.REVERSE);

        pixelKicker = opMode.hardwareMap.get(Servo.class, "pixelKicker");
        pixelKicker.setPosition(1);

        sonarLeft = opMode.hardwareMap.get(AnalogInput.class, "sonarLeft"); //TODO remove sonarLeft from teleop

        sonarRight = opMode.hardwareMap.get(AnalogInput.class, "sonarRight"); //TODO remove sonarLeft from teleop

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
        opMode.sleep(8000);


//        dashboardTelemetry.addData("detect attempts", detectAttemptCount);
//        dashboardTelemetry.addData("errorX", errorX);
//        dashboardTelemetry.update();
//
    }
}