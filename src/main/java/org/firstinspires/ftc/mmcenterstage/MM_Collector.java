package org.firstinspires.ftc.mmcenterstage;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
public class MM_Collector {
    private final LinearOpMode opMode;

    public static double RIGHT_WHEEL_POWER = 1;
    public static double LEFT_WHEEL_POWER = 0.5;
    public static double PIXEL_PRECISION_WHEEL_POWER = 1;

    private CRServo rightWheel = null;
    private DcMotorEx leftWheel = null;
    private CRServo pixelPrecisionWheel = null;

    boolean isDual = true;

    public MM_Collector(LinearOpMode opMode) {
        this.opMode = opMode;
        init();
    }

    public void collect() {
        if (opMode.gamepad2.left_bumper) {
            leftWheel.setPower(LEFT_WHEEL_POWER);
            rightWheel.setPower(RIGHT_WHEEL_POWER);
            pixelPrecisionWheel.setPower(-PIXEL_PRECISION_WHEEL_POWER);
        } else if (opMode.gamepad2.left_trigger > 0.1) {
            leftWheel.setPower(-LEFT_WHEEL_POWER);
            rightWheel.setPower(-RIGHT_WHEEL_POWER);
            pixelPrecisionWheel.setPower(PIXEL_PRECISION_WHEEL_POWER);
        } else {
            leftWheel.setPower(0);
            rightWheel.setPower(0);
            pixelPrecisionWheel.setPower(0);
        }
    }

    public void score(){
        pixelPrecisionWheel.setPower(-1);
        opMode.sleep(2000);
        pixelPrecisionWheel.setPower(0);
    }

    public void init() {
        rightWheel = opMode.hardwareMap.get(CRServo.class, "rightWheel");
        leftWheel = opMode.hardwareMap.get(DcMotorEx.class, "leftWheel");
        pixelPrecisionWheel = opMode.hardwareMap.get(CRServo.class, "innerWheel");

        leftWheel.setDirection(DcMotorEx.Direction.REVERSE);
        pixelPrecisionWheel.setDirection(CRServo.Direction.REVERSE);
    }
}
