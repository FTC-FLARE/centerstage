package org.firstinspires.ftc.mmcenterstage;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class MM_Transport {
    private final LinearOpMode opMode;
    private final Telemetry dashboardTelemetry;

    private DcMotorEx slide = null;
    public Servo boxFlip = null;
    private TouchSensor bottomLimit = null;

    public static int TICK_INCREMENT = 30;
    public static double BOX_COLLECT = .375;
    public static double BOX_SCORE = 639;
    public static double BOX_TRANSPORT = .357;
    public static final int UPPER_LIMIT = 2900;
    public static final int MIN_SCORE_HEIGHT = 1560;
    public static final int MAX_COLLECT_HEIGHT = 350;

    boolean readyToScore = false;
    boolean isLimitHandled = false;
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


//        if (rightStickPower > 0.1) {
//            targetTicks = Math.min(targetTicks + TICK_INCREMENT, UPPER_LIMIT);
//
//        }
//        } else if(bottomLimit.isPressed() && !isLimitHandled) {
//
//            slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//            slide.setPower(0);
//            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            isLimitHandled = true;
//        } else if (!bottomLimit.isPressed()) {
//            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            slide.setPower(1);
//            isLimitHandled = !isLimitHandled;
//
//        }
//        slide.setTargetPosition(targetTicks);
//        slide.setPower(1);

        if (bottomLimit.isPressed() && !isLimitHandled){
            slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            slide.setPower(0);
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slide.setTargetPosition(0);
            isLimitHandled = true;
        } else if (!bottomLimit.isPressed() || rightStickPower > 0.1) {
            if (rightStickPower < -0.1) {
                targetTicks = Math.max(targetTicks - TICK_INCREMENT, 0);
            } else if (rightStickPower > 0.1) {
                targetTicks = Math.min(targetTicks + TICK_INCREMENT, UPPER_LIMIT);
            }

            slide.setTargetPosition(targetTicks);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setPower(1);

            isLimitHandled = false;
        }

        if (slide.getCurrentPosition() > MIN_SCORE_HEIGHT) {
            if (!MM_TeleOp.previousGamepad2.right_stick_button && MM_TeleOp.currentGamepad2.right_stick_button) {
                readyToScore = !readyToScore;
                boxFlip.setPosition((readyToScore) ? BOX_SCORE : BOX_TRANSPORT);
            }
        } else {
            boxFlip.setPosition((slide.getCurrentPosition() > MAX_COLLECT_HEIGHT) ? BOX_TRANSPORT : BOX_COLLECT);
        }

    }

    public void init() {
        slide = opMode.hardwareMap.get(DcMotorEx.class, "slide");
        boxFlip = opMode.hardwareMap.get(Servo.class, "boxFlip");
        bottomLimit = opMode.hardwareMap.get(TouchSensor.class, "bottomLimit");

        slide.setDirection(DcMotorEx.Direction.REVERSE);

        slide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        boxFlip.setPosition(BOX_COLLECT);
    }
}
