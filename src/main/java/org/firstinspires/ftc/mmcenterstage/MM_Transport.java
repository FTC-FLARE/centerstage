package org.firstinspires.ftc.mmcenterstage;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class MM_Transport {
    private final LinearOpMode opMode;
    private final Telemetry dashboardTelemetry;

    private DcMotorEx slide = null;
    private Servo boxFlip = null;
    //add lower limit

    public static int TICK_INCREMENT = 5;
    int targetTicks = 0;
    public static int upperLimit = 3000;

    public MM_Transport(LinearOpMode opMode, Telemetry dashboardTelemetry) {
        this.opMode = opMode;
        this.dashboardTelemetry = dashboardTelemetry;
        init();
    }

    public void transport() {
        dashboardTelemetry.addData("slide pos", slide.getCurrentPosition());
        dashboardTelemetry.update();

        if (opMode.gamepad2.right_trigger > 0.1 && slide.getCurrentPosition() < upperLimit) {
            targetTicks = Math.min(targetTicks + TICK_INCREMENT, upperLimit);
        } else if (opMode.gamepad2.left_trigger > 0.1 && slide.getCurrentPosition() > 0) {
            targetTicks = Math.max(targetTicks - TICK_INCREMENT, 0);
        }
        slide.setTargetPosition(targetTicks);
        slide.setPower(1);
    }

    public void init() {
        slide = opMode.hardwareMap.get(DcMotorEx.class, "slide");
        boxFlip = opMode.hardwareMap.get(Servo.class, "boxFlip");

        slide.setDirection(DcMotorSimple.Direction.REVERSE);

        slide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }
}
