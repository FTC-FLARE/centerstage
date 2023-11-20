package org.firstinspires.ftc.mmcenterstage;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class MM_Transport {
    private final LinearOpMode opMode;
    private final Telemetry dashboardTelemetry;

    private DcMotorEx slide = null;
    public Servo boxFlip = null;
    //add lower limit

    public static int TICK_INCREMENT = 5;
    public static double BOX_COLLECT = .42;
    public static double BOX_SCORE = 1;
    public static final int UPPER_LIMIT = 2900;
    public static final int MIN_SCORE_HEIGHT = 1450;
    public static final int MAX_COLLECT_HEIGHT = 350;

    boolean readyToScore = false;
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

        if (rightStickPower > 0.1) {
            targetTicks = Math.min(targetTicks + TICK_INCREMENT, UPPER_LIMIT);
        } else if (rightStickPower < -0.1 && slide.getCurrentPosition() > 0) {
            targetTicks = Math.max(targetTicks - TICK_INCREMENT, 0);
        }
        slide.setTargetPosition(targetTicks);
        slide.setPower(1);

        if (slide.getCurrentPosition() > MIN_SCORE_HEIGHT) {
            if (!MM_TeleOp.previousGamepad2.right_stick_button && MM_TeleOp.currentGamepad2.right_stick_button) {
                readyToScore = !readyToScore;
                boxFlip.setPosition((readyToScore) ? BOX_SCORE : 0);
            }
        } else {
            boxFlip.setPosition((slide.getCurrentPosition() > MAX_COLLECT_HEIGHT) ? 0 : BOX_COLLECT);
        }
    }

    public void init() {
        slide = opMode.hardwareMap.get(DcMotorEx.class, "slide");
        boxFlip = opMode.hardwareMap.get(Servo.class, "boxFlip");

        slide.setDirection(DcMotorEx.Direction.REVERSE);

        slide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        boxFlip.setPosition(BOX_COLLECT);
    }
}
