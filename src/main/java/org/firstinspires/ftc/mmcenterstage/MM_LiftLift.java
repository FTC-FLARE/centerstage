package org.firstinspires.ftc.mmcenterstage;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class MM_LiftLift {
    MM_OpMode opMode;

//    public MM_Launch launch;

    //    private DcMotorEx liftArm = null;
    private DcMotorEx robotLift = null;

    private Servo liftRelease = null;

    private final ElapsedTime dangerTimer = new ElapsedTime();

    static final int LAUNCH_POS_TARGET_TICKS = 1000;
    public static final int HANG_POS_TARGET_TICKS = 200;
    public static final double SAFETY_TIME = 1.5;
    static final int TICK_INCREMENT = 12;
    public static double LIFT_RELEASE = 0;
    public static double LIFT_LOCK_POS = 0.23;

    int LSClickCount = 0;
    public static boolean isInLaunchPos = false;
    boolean liftIsVertical = false;
    int targetTicks = 0;

    public MM_LiftLift(MM_OpMode opMode) {
        this.opMode = opMode;
        init();
    }

    public void liftLift() {
//        if (MM_TeleOp.matchTimer.time() > 30)
//            if (!MM_TeleOp.previousGamepad2.left_stick_button && MM_TeleOp.currentGamepad2.left_stick_button) {
//                dangerTimer.reset();
//            }
//
//
//        if (opMode.gamepad2.dpad_up) {
//            liftArm.setPower(.3);
//            isInHangPos = true;
//        } else if (opMode.gamepad2.dpad_down) {
//            liftArm.setPower(-.3);
//        } else {
//            liftArm.setPower(0);
//        }
//
//        //if (liftArm.getCurrentPosition() > 100) {
//            if (-opMode.gamepad2.left_stick_y > 0.1) {
//                robotLift.setPower(1);
//            } else if (-opMode.gamepad2.left_stick_y < -0.1) {
//                robotLift.setPower(-1);
//            } else {
//                robotLift.setPower(0);
//            }
//        //}
//        if (opMode.getRuntime() >= 120) {      // ***** Temp comment out *****
            if (opMode.gamepad1.right_bumper) {
                liftRelease.setPosition(LIFT_RELEASE);
                liftIsVertical = true;
            }
            if (liftIsVertical) {
                if (opMode.gamepad1.right_trigger > 0.1) {
                    robotLift.setPower(1);
                } else if (opMode.gamepad1.left_trigger > 0.1) {
                    robotLift.setPower(-1);
                } else {
                    robotLift.setPower(0);
                }
            }
//        }

        opMode.multipleTelemetry.addData("robot lift current ticks", robotLift.getCurrentPosition());
//        dashboardTelemetry.addData("lift lift current ticks", liftArm.getCurrentPosition());
        opMode.multipleTelemetry.addData("danger timer", dangerTimer.time());
        opMode.multipleTelemetry.update();
    }

    public void init() {
//        liftArm = opMode.hardwareMap.get(DcMotorEx.class, "liftArm");
        robotLift = opMode.hardwareMap.get(DcMotorEx.class, "robotLift");

//        launch = new MM_Launch(opMode);

//        liftArm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        liftArm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        robotLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robotLift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        liftRelease = opMode.hardwareMap.get(Servo.class, "liftRelease");
        liftRelease.setPosition(LIFT_LOCK_POS);
    }
}
