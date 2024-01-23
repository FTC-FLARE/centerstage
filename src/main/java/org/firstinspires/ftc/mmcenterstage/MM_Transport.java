package org.firstinspires.ftc.mmcenterstage;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Config
public class MM_Transport {
    private final MM_OpMode opMode;

    private DcMotorEx slide = null;
    private DcMotorEx boxFlip = null;
    private TouchSensor bottomLimit = null;
//    private DistanceSensor boxSensor = null;

    public static int SLIDE_TICK_INCREMENT = 30;

    public static final int UPPER_LIMIT = 2900;
    public static final int MIN_SCORE_HEIGHT = 1560;
    public static final int SLIDE_SLOW_DOWN_TICKS = 800;
    public static final int BOX_SCORE_TICKS = 380;
    public static final int BOX_TICK_INCREMENT = 8;
    public static final double BOX_FLIP_POWER = 0.52;
    public static final double SLIDE_HOME_POWER = -0.7;
    public static final double SLIDE_HOME_POWER_SLOW = -0.3;

    boolean readyToScore = false;
    boolean isLimitHandled = false;
    boolean isHoming = false;
    double rightStickPower = 0;  // slide
    double leftStickPower = 0;  // box flip
    int slideTargetTicks = 0;
    int boxFlipTargetTicks = 0;

    public MM_Transport(MM_OpMode opMode) {
        this.opMode = opMode;
        init();
    }

    public void control() {
        opMode.multipleTelemetry.addData("slide pos", slide.getCurrentPosition());
        opMode.multipleTelemetry.update();

        rightStickPower = -opMode.gamepad2.right_stick_y;
        leftStickPower = -opMode.gamepad2.left_stick_y;

        if (rightStickPower > .1) {
            isHoming = false;
        }

        if (bottomLimit.isPressed() && !isLimitHandled) {
            // reset slide encoder
            slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            slide.setPower(0);
            slide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            slideTargetTicks = 0;

            // reset box flip encoder
            boxFlip.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            boxFlip.setPower(0);
            boxFlip.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            boxFlipTargetTicks = 0;

            isLimitHandled = true;
            isHoming = false;
        } else if (!isHoming) {
            if ((MM_TeleOp.currentGamepad2.y && !MM_TeleOp.previousGamepad2.y) && !bottomLimit.isPressed()) { // start homing
                boxFlip.setTargetPosition(0);
                boxFlipTargetTicks = 0;
                readyToScore = false;

                slide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                slide.setPower(slide.getCurrentPosition() < SLIDE_SLOW_DOWN_TICKS ? SLIDE_HOME_POWER_SLOW : SLIDE_HOME_POWER);
                isHoming = true;
            } else if (!bottomLimit.isPressed() || rightStickPower > 0.1) { // limit not triggered or i'm trying to go up
                if (rightStickPower < -0.1) {
                    slideTargetTicks = Math.max(slideTargetTicks - SLIDE_TICK_INCREMENT, 0);
                } else if (rightStickPower > 0.1) {
                    slideTargetTicks = Math.min(slideTargetTicks + SLIDE_TICK_INCREMENT, UPPER_LIMIT);
                }

                slide.setTargetPosition(slideTargetTicks);
                slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                slide.setPower(1);
                if (!bottomLimit.isPressed()) {
                    isLimitHandled = false;
                }
            }
        }

        if (slide.getCurrentPosition() > MIN_SCORE_HEIGHT && !MM_TeleOp.previousGamepad2.right_stick_button && MM_TeleOp.currentGamepad2.right_stick_button) {
            readyToScore = !readyToScore;
            boxFlipTargetTicks = (readyToScore) ? BOX_SCORE_TICKS : 0;
        }
        if (!bottomLimit.isPressed()) {
            if (leftStickPower > 0.1) {
                boxFlipTargetTicks += BOX_TICK_INCREMENT;
            } else if (leftStickPower < -0.1) {
                boxFlipTargetTicks -= BOX_TICK_INCREMENT;
            }

            boxFlip.setTargetPosition(boxFlipTargetTicks);
            boxFlip.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            boxFlip.setPower(BOX_FLIP_POWER);
        } else {
            boxFlip.setPower(0);
        }
        opMode.multipleTelemetry.addData("Box pos", boxFlip.getCurrentPosition());
    }

    public void runToScorePos() {
        boxFlip.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        boxFlip.setTargetPosition(0);
        boxFlip.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        boxFlip.setPower(BOX_FLIP_POWER);

        slide.setTargetPosition(MIN_SCORE_HEIGHT);
        slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        slide.setPower(.5);
        while (slide.isBusy()) {
            opMode.telemetry.addData("Slide running", slide.getCurrentPosition());
            opMode.telemetry.update();
        }
        boxFlip.setTargetPosition(BOX_SCORE_TICKS);

        while (boxFlip.isBusy()) {
            opMode.telemetry.addData("Box Flipping", boxFlip.getCurrentPosition());
            opMode.telemetry.update();
        }
    }

    public void goHome() {
        boxFlip.setTargetPosition(0);
        while (boxFlip.isBusy()) {
        }
        slide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slide.setPower(slide.getCurrentPosition() < SLIDE_SLOW_DOWN_TICKS ? SLIDE_HOME_POWER_SLOW : SLIDE_HOME_POWER);
        while (!bottomLimit.isPressed()) {
        }
        slide.setPower(0);
    }

    public void init() {
        slide = opMode.hardwareMap.get(DcMotorEx.class, "slide");

        bottomLimit = opMode.hardwareMap.get(TouchSensor.class, "bottomLimit");
        //boxSensor = opMode.hardwareMap.get(DistanceSensor.class, "boxSensor");

        slide.setDirection(DcMotorEx.Direction.REVERSE);

        slide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        boxFlip = opMode.hardwareMap.get(DcMotorEx.class, "mtrBoxFlip");
        boxFlip.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        boxFlip.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        boxFlip.setPower(0);
    }
}
