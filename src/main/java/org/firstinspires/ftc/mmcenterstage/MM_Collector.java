package org.firstinspires.ftc.mmcenterstage;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class MM_Collector {
    private final LinearOpMode opMode;

    private DcMotorEx leftWheel = null;
    private DcMotorEx rightWheel = null;

    public MM_Collector(LinearOpMode opMode) {
        this.opMode = opMode;
        init();
    }

    public void collect() {
        if (opMode.gamepad2.right_trigger > 0.1) {
            leftWheel.setPower(-opMode.gamepad2.right_trigger);
            rightWheel.setPower(-opMode.gamepad2.right_trigger);
        } else if (opMode.gamepad2.left_trigger > 0.1) {
            leftWheel.setPower(opMode.gamepad2.left_trigger);
            rightWheel.setPower(opMode.gamepad2.left_trigger);
        } else {
            leftWheel.setPower(0);
            rightWheel.setPower(0);
        }
    }

    public void init() {
        leftWheel = opMode.hardwareMap.get(DcMotorEx.class, "leftWheel");
        rightWheel = opMode.hardwareMap.get(DcMotorEx.class, "rightWheel");

        rightWheel.setDirection(DcMotorEx.Direction.REVERSE);
    }
}
