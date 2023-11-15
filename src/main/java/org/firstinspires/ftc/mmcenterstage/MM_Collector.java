package org.firstinspires.ftc.mmcenterstage;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

public class MM_Collector {
    private final LinearOpMode opMode;

    private CRServo leftWheel = null;
    private DcMotorEx rightWheel = null;
    private CRServo pixelPrecisionWheel = null;

    private Gamepad currentGamepad2Collect;
    private Gamepad previousGamepad2Collect;

    boolean isDual = true;

    public MM_Collector(LinearOpMode opMode, Gamepad currentGamepad2Collect, Gamepad previousGamepad2Collect) {
        this.opMode = opMode;
        this.currentGamepad2Collect = currentGamepad2Collect;
        this.previousGamepad2Collect = previousGamepad2Collect;
        init();
    }

    public void collect() {
        previousGamepad2Collect.copy(currentGamepad2Collect);
        currentGamepad2Collect.copy(opMode.gamepad2);

        if (previousGamepad2Collect.dpad_up != currentGamepad2Collect.dpad_up) {
            isDual = !isDual;
        }

        if (opMode.gamepad2.right_bumper && isDual) { //set power to both wheels spinning in
            leftWheel.setPower(1);
            rightWheel.setPower(0.5);
            pixelPrecisionWheel.setPower(1);
        } else if (opMode.gamepad2.right_bumper && !isDual) { //set power to right wheel spinning in
            leftWheel.setPower(0);
            rightWheel.setPower(0.5);
            pixelPrecisionWheel.setPower(1);
        } else if (opMode.gamepad2.left_bumper) { //set power to both wheels spinning out
            leftWheel.setPower(-1);
            rightWheel.setPower(-0.5);
            pixelPrecisionWheel.setPower(-1);
        } else {
            leftWheel.setPower(0);
            rightWheel.setPower(0);
            pixelPrecisionWheel.setPower(0);
        }
    }

    public void init() {
        leftWheel = opMode.hardwareMap.get(CRServo.class, "leftWheel");
        rightWheel = opMode.hardwareMap.get(DcMotorEx.class, "rightWheel");
        pixelPrecisionWheel = opMode.hardwareMap.get(CRServo.class, "innerWheel");

        rightWheel.setDirection(DcMotorEx.Direction.REVERSE);
    }
}
