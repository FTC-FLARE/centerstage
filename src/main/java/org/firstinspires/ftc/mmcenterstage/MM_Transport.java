package org.firstinspires.ftc.mmcenterstage;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class MM_Transport {
    private final LinearOpMode opMode;

    private DcMotorEx slide = null;
    private Servo boxFlip = null;

    private static double TICK_INCREMENT = 5;
    int targetTicks = 0;
    static double encoderLimit = 1000; //change after telemetry

    public MM_Transport (LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void transport() {
        if (opMode.gamepad2.right_trigger > 0.1 && slide.getCurrentPosition() < targetTicks) {
            slide.setTargetPosition(targetTicks);
            slide.setPower(opMode.gamepad2.right_trigger);
            targetTicks += TICK_INCREMENT;
        } else if (opMode.gamepad2.left_trigger > 0.1) {
            slide.setTargetPosition(targetTicks);
            slide.setPower(-opMode.gamepad2.left_trigger);
            targetTicks -= TICK_INCREMENT;
        } else {
            slide.setTargetPosition(targetTicks);
            slide.setPower(0.3);
        }
    }

    public void init() {
        slide = opMode.hardwareMap.get(DcMotorEx.class, "slide");
        boxFlip = opMode.hardwareMap.get(Servo.class, "boxFlip");

        slide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
