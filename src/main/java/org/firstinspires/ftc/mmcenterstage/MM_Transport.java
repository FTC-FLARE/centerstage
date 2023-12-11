package org.firstinspires.ftc.mmcenterstage;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class MM_Transport {
    private final LinearOpMode opMode;
    private final Telemetry dashboardTelemetry;

    private DcMotorEx slide = null;
//    public Servo boxFlip = null;
    private DcMotorEx mtrBoxFlip = null;
    private TouchSensor bottomLimit = null;
//    private DistanceSensor boxSensor = null;

    public static int TICK_INCREMENT = 30;
    public static double BOX_COLLECT = .14;
    public static double BOX_SCORE = .41;
    public static double BOX_TRANSPORT = .1256;
    public static final int UPPER_LIMIT = 2900;
    public static final int MIN_SCORE_HEIGHT = 1560;
    public static final int MAX_COLLECT_HEIGHT = 350;
    public static final int MTR_BOX_SCORE = 300;

    boolean readyToScore = false;
    boolean isLimitHandled = false;
    boolean isHoming = false;
    double rightStickPower = 0;
    int targetTicks = 0;

    public MM_Transport(LinearOpMode opMode, Telemetry dashboardTelemetry) {
        this.opMode = opMode;
        this.dashboardTelemetry = dashboardTelemetry;
        init();
    }

    public void transport() {
        dashboardTelemetry.addData("slide pos", slide.getCurrentPosition());
        dashboardTelemetry.update();

        rightStickPower = -opMode.gamepad2.right_stick_y;

        if(rightStickPower > .1){
            isHoming = false;
        }

        if (bottomLimit.isPressed() && !isLimitHandled) {
            slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            slide.setPower(0);
            slide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            targetTicks = 0;
            isLimitHandled = true;
            isHoming = false;
        }  else if ((MM_TeleOp.currentGamepad2.y && !MM_TeleOp.previousGamepad2.y) && !bottomLimit.isPressed()) {
            slide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            slide.setPower(-.6);
            isHoming = true;
        } else if ((!bottomLimit.isPressed() || rightStickPower > 0.1) && !isHoming) {// not trigger or i'm trying to go up
            if (rightStickPower < -0.1) {
                targetTicks = Math.max(targetTicks - TICK_INCREMENT, 0);
            } else if (rightStickPower > 0.1) {
                targetTicks = Math.min(targetTicks + TICK_INCREMENT, UPPER_LIMIT);
            }

            slide.setTargetPosition(targetTicks);
            slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            slide.setPower(1);

            isLimitHandled = false;
        }

        if (slide.getCurrentPosition() > MIN_SCORE_HEIGHT) {
            if (!MM_TeleOp.previousGamepad2.right_stick_button && MM_TeleOp.currentGamepad2.right_stick_button) {
                readyToScore = !readyToScore;
//                boxFlip.setPosition((readyToScore) ? BOX_SCORE : BOX_TRANSPORT);
                mtrBoxFlip.setTargetPosition((readyToScore) ? MTR_BOX_SCORE : 0);
            }
        } else {
//            boxFlip.setPosition((slide.getCurrentPosition() > MAX_COLLECT_HEIGHT) ? BOX_TRANSPORT : BOX_COLLECT);
            mtrBoxFlip.setTargetPosition(0);
        }
        dashboardTelemetry.addData("Box pos", mtrBoxFlip.getCurrentPosition());
    }

    public void init() {
        slide = opMode.hardwareMap.get(DcMotorEx.class, "slide");
//        boxFlip = opMode.hardwareMap.get(Servo.class, "boxFlip");

        bottomLimit = opMode.hardwareMap.get(TouchSensor.class, "bottomLimit");
        //boxSensor = opMode.hardwareMap.get(DistanceSensor.class, "boxSensor");

        slide.setDirection(DcMotorEx.Direction.REVERSE);

        slide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        mtrBoxFlip = opMode.hardwareMap.get(DcMotorEx.class, "mtrBoxFlip");
        mtrBoxFlip.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        mtrBoxFlip.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        mtrBoxFlip.setTargetPosition(0);
        mtrBoxFlip.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        mtrBoxFlip.setPower(.5);

//        boxFlip.setPosition(BOX_COLLECT);
    }
}
