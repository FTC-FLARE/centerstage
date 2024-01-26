package org.firstinspires.ftc.mmcenterstage;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
public class MM_Drivetrain {
    private final MM_OpMode opMode;

    private DcMotorEx flMotor = null;
    private DcMotorEx frMotor = null;
    private DcMotorEx blMotor = null;
    private DcMotorEx brMotor = null;
    private Servo pixelKicker = null;
    public MM_VisionPortal visionPortal;
    private IMU imu;

    private final ElapsedTime timer = new ElapsedTime();
    ElapsedTime kickTime = new ElapsedTime();

    public static double MAX_DRIVE_POWER = .5;
    public static double MIN_DRIVE_POWER = .28;
    public static double DRIVE_P_COEFF = .0166;

    public static double STRAFE_P_COEFF = .02;

    public static double MAX_TURN_POWER = .5;
    public static double MIN_TURN_POWER = .15;
    public static double GYRO_TURN_P_COEFF = .016;
    public static double APRIL_TAG_TURN_P_COEFF = .004;
    public static double HEADING_ERROR_THRESHOLD = 2;

    public static double APRIL_TAG_ERROR_THRESHOLD = 2;
    public static double APRIL_TAG_ERROR_THRESHOLD_YAW = 7;


    public static double MAX_DETECT_ATTEMPTS = 250;

    public final double WHEEL_DIAMETER = 4;
    public final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    public final double TICKS_PER_REVOLUTION = 537.7; // for drivetrain only(5202-0002-0027 "753.2" TPR), change for the real robot ("537.7" TPR)
    public final double TICKS_PER_INCH = TICKS_PER_REVOLUTION / WHEEL_CIRCUMFERENCE;

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
    private double aprilTagErrorX = 0;
    private double aprilTagErrorY = 0;
    private double aprilTagErrorYaw = 0;

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

        flMotor.setTargetPosition(ticks + flMotor.getCurrentPosition());
        frMotor.setTargetPosition(ticks + frMotor.getCurrentPosition());
        blMotor.setTargetPosition(ticks + blMotor.getCurrentPosition());
        brMotor.setTargetPosition(ticks + brMotor.getCurrentPosition());

        setDriveMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        setDrivePowers(power);
        while(opMode.opModeIsActive() && (flMotor.isBusy() || brMotor.isBusy())){ }
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

    public void rotateToAngle(int targetAngle) {
        setDriveMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double error = getYawError(targetAngle, heading);

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
            error = getYawError(targetAngle, heading);

            opMode.multipleTelemetry.addData("power", power);
            opMode.multipleTelemetry.addData("error", error);
            opMode.multipleTelemetry.addData("heading", heading);
            opMode.multipleTelemetry.update();
        }
        setDrivePowers(0);
    }

    public boolean driveToAprilTag(int tagToFind, double targetX, double targetY, double targetYaw) {
        setDriveMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        boolean keepGoing = true;
        detectAttemptCount = 0;
        double drivePower = 0;
        double strafePower = 0;
        double rotatePower = 0;
        AprilTagDetection tagInfo = null;

        while (opMode.opModeIsActive() && keepGoing) {
            tagInfo = visionPortal.getAprilTagInfo(tagToFind);

            if (tagInfo != null) {
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
                detectAttemptCount++;
                if (detectAttemptCount >= MAX_DETECT_ATTEMPTS) {//TODO if tag not found
                    setDrivePowers(0);
                    opMode.multipleTelemetry.addLine("lost aprilTag");
                    opMode.multipleTelemetry.update();

                    return false;
                }
                opMode.sleep(2);
            }

            opMode.multipleTelemetry.addData("errorY", aprilTagErrorY);
            opMode.multipleTelemetry.addData("errorX", aprilTagErrorX);
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

    public void cruiseUnderTruss(){   //DO NOT RENAME; IF RENAMED THIS WILL BECOME A WAR!!!
    // TODO cruise under truss
    }

    public int purplePixelLeft(){
        int propPos = visionPortal.propPositionLeft();

        if (propPos == 0){
            driveInches(-20, 0.5);
            rotateToAngle(45);
            driveInches(-12, 0.5);
            rotateToAngle(0);
            driveInches(10, 0.5);
            rotateToAngle(90);
            if (MM_OpMode.alliance == MM_OpMode.BLUE){
                MM_OpMode.foundApriltagScoreYellow = driveToAprilTag(1, 1, 3.4, 0);
            }
        } else if (propPos == 1){
            driveInches(-32, 0.5);
            driveInches(10, 0.5);
            rotateToAngle(90);
            if (MM_OpMode.alliance == MM_OpMode.BLUE) {
                MM_OpMode.foundApriltagScoreYellow = driveToAprilTag(2, 1, 3.4, 0);
            }
        } else {
            driveInches(-20, 0.5);
            rotateToAngle(-45);
            driveInches(-11, 0.5);
            driveInches(10, 0.5);
            rotateToAngle(90);
            if (MM_OpMode.alliance == MM_OpMode.BLUE) {
                MM_OpMode.foundApriltagScoreYellow = driveToAprilTag(3, 1, 3.4, 0);
            }
        }
        return propPos;
    }

    public int purplePixelRight(){
        int propPos = visionPortal.propPositionRight();

        if (propPos == 0){
            driveInches(-20, 0.5);
            rotateToAngle(45);
            driveInches(-12, 0.5);
            driveInches(15, 0.5);
            rotateToAngle(0);
            driveInches(-10, 0.5);
            rotateToAngle(-90);
            if (MM_OpMode.alliance == MM_OpMode.RED){
                driveToAprilTag(4, 1, 3.4, 0);
            }
        } else if (propPos == 1){
            driveInches(-32, 0.5);
            driveInches(10, 0.5);
            rotateToAngle(-90);
            if (MM_OpMode.alliance == MM_OpMode.RED) {
                driveToAprilTag(5, 1, 3.4, 0);
            }
        } else {
            driveInches(-20, 0.5);
            rotateToAngle(-45);
            driveInches(-12, 0.5);
            rotateToAngle(0);
            driveInches(10, 0.5);
            rotateToAngle(-90);
            if (MM_OpMode.alliance == MM_OpMode.RED) {
                driveToAprilTag(6, 1, 3.4, 0);
            }
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

    private double getYawError(int targetAngle, double currentAngle) {
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