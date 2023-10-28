package org.firstinspires.ftc.mmcenterstage;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public class MM_Drivetrain {
    private final LinearOpMode opMode;
    private final double APRIL_TAG_THRESHOLD = 2;
    private final double DRIVE_P_COEFF = .09654;
    private final double MAX_DRIVE_POWER = .7;

    private DcMotorEx flMotor = null;
    private DcMotorEx frMotor = null;
    private DcMotorEx blMotor = null;
    private DcMotorEx brMotor = null;

    public MM_AprilTags aprilTags;

    private Gamepad currentGamepad1;
    private Gamepad previousGamepad1;
    boolean isSlow = false;

    public MM_Drivetrain(LinearOpMode opMode, Gamepad currentGamepad1, Gamepad previousGamepad1) {
        this.opMode = opMode;
        this.currentGamepad1 = currentGamepad1;
        this.previousGamepad1 = previousGamepad1;
        init();
    }

    public void driveWithSticks() {
        double drivePower = -opMode.gamepad1.left_stick_y;
        double strafePower = opMode.gamepad1.left_stick_x;
        double rotatePower = opMode.gamepad1.right_stick_x;

        double FLPower = drivePower + strafePower + rotatePower;
        double FRPower = drivePower - strafePower - rotatePower;
        double BLPower = drivePower - strafePower + rotatePower;
        double BRPower = drivePower + strafePower - rotatePower;

        if (currentGamepad1.a && !previousGamepad1.a) {
            isSlow = !isSlow;
        }

        double maxMotorPwr = Math.max(Math.max(Math.abs(FLPower), Math.abs(FRPower)),
                Math.max(Math.abs(BLPower), Math.abs(BRPower)));

        if (maxMotorPwr > 1) {
            FLPower /= maxMotorPwr;
            FRPower /= maxMotorPwr;
            BLPower /= maxMotorPwr;
            BRPower /= maxMotorPwr;
        }

        if (isSlow) {
            FLPower *= 0.5;
            FRPower *= 0.5;
            BLPower *= 0.5;
            BRPower *= 0.5;
        }

        flMotor.setPower(FLPower * 0.7);
        frMotor.setPower(FRPower * 0.7);
        blMotor.setPower(BLPower * 0.7);
        brMotor.setPower(BRPower * 0.7);

        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(opMode.gamepad1);
    }

    public void driveToAprilTag() {
        opMode.telemetry.addData("error", getError("y",3, 2));

        while (opMode.opModeIsActive() && getError("y", 3, 2) > APRIL_TAG_THRESHOLD){
            double power = Math.abs(getError("y", 3, 2) * DRIVE_P_COEFF * MAX_DRIVE_POWER);
            opMode.telemetry.addData("power", power);
            opMode.telemetry.update();
            flMotor.setPower(power);
            frMotor.setPower(power);
            blMotor.setPower(power);
            brMotor.setPower(power);
        }
    }

    private double getError(String axis, double target, int targetId) {
       double error = 0;

        AprilTagDetection tagId = getId(targetId);

        if (axis.equals("y") && tagId != null)  {
           error = target - tagId.ftcPose.y;
           return Math.abs(error);
       } else {
           opMode.telemetry.addData("no aprilTags", "detected");
       }
       if(axis.equals("x")){
           error = target - tagId.ftcPose.x;
       }
       return Math.abs(error);
    }

    public AprilTagDetection getId(int id) {
        List<AprilTagDetection> currentDetections = aprilTags.aprilTagProcessor.getDetections();
        opMode.telemetry.addData("list size", currentDetections.size());

        for(AprilTagDetection detection : currentDetections) {
            if(detection.id == id){
                opMode.telemetry.addData("id", detection.id);
                return detection;
            }

        }

        return null;
    }


    public void init(){
        flMotor = opMode.hardwareMap.get(DcMotorEx.class, "flMotor");
        frMotor = opMode.hardwareMap.get(DcMotorEx.class, "frMotor");
        blMotor = opMode.hardwareMap.get(DcMotorEx.class, "blMotor");
        brMotor = opMode.hardwareMap.get(DcMotorEx.class, "brMotor");

        flMotor.setDirection(DcMotorEx.Direction.REVERSE);
        blMotor.setDirection(DcMotorEx.Direction.REVERSE);

        aprilTags = new MM_AprilTags(opMode);
    }
}

