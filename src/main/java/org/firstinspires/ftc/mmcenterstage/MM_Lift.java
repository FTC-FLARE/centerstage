package org.firstinspires.ftc.mmcenterstage;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class MM_Lift {
    MM_OpMode opMode;

    private DcMotorEx lift = null;
    private Servo liftRelease = null;

    public static double LIFT_RELEASE_POS = 0;
    public static double LIFT_LOCK_POS = 0.23;

    public MM_Lift(MM_OpMode opMode) {
        this.opMode = opMode;
        init();
    }

    public void release(){
        liftRelease.setPosition(LIFT_RELEASE_POS);
    }

    public void control() {
            if (opMode.gamepad1.right_trigger > 0.1) {
                lift.setPower(1);
            } else if (opMode.gamepad1.left_trigger > 0.1) {
                lift.setPower(-1);
            } else {
                lift.setPower(0);
            }
    }

    public void init() {
        lift = opMode.hardwareMap.get(DcMotorEx.class, "robotLift");
        lift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        liftRelease = opMode.hardwareMap.get(Servo.class, "liftRelease");
        liftRelease.setPosition(LIFT_LOCK_POS);
    }
}
