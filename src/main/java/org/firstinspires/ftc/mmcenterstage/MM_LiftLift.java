//package org.firstinspires.ftc.mmcenterstage;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//
//@Config
//public class MM_LiftLift {
//    LinearOpMode opMode;
//
//    public MM_Launch launch;
//
//    private DcMotorEx liftArm = null;
//    private DcMotorEx robotLift = null;
//
//    private final ElapsedTime dangerTimer = new ElapsedTime();
//    Telemetry dashboardTelemetry;
//
//    static final int LAUNCH_POS_TARGET_TICKS = 1000;
//    static final int HANG_POS_TARGET_TICKS = 2000;
//    public static final double SAFETY_TIME = 1.5;
//    static final int TICK_INCREMENT = 12;
//
//    int LSClickCount = 0;
//    public static boolean isInLaunchPos = false;
//    boolean isInHangPos = false;
//    int targetTicks = 0;
//
//    public MM_LiftLift(LinearOpMode opMode, Telemetry dashboardTelemetry) {
//        this.opMode = opMode;
//        this.dashboardTelemetry = dashboardTelemetry;
//        init();
//    }
//
//    public void liftLift() {
//        if (!MM_TeleOp.previousGamepad2.left_stick_button && MM_TeleOp.currentGamepad2.left_stick_button) {
//            dangerTimer.reset();
//        }
//
//        if (MM_TeleOp.currentGamepad2.left_stick_button && !MM_TeleOp.previousGamepad2.left_stick_button && LSClickCount == 0) {
//            liftArm.setTargetPosition(LAUNCH_POS_TARGET_TICKS);
//            liftArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//            liftArm.setPower(1);
//            LSClickCount++;
//            isInLaunchPos = true;
//        } else if (LSClickCount == 1 && opMode.gamepad2.left_stick_button && dangerTimer.time() > SAFETY_TIME) {
//            liftArm.setTargetPosition(HANG_POS_TARGET_TICKS);
//            liftArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//            isInHangPos = true;
//        }
//
//        if (isInHangPos) {
//            if (-opMode.gamepad2.left_stick_y > 0.1) {
//                targetTicks += TICK_INCREMENT;
//                robotLift.setTargetPosition(targetTicks);
//                robotLift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//                robotLift.setPower(1);
//            } else if (-opMode.gamepad2.left_stick_y < -0.1) {
//                targetTicks -= TICK_INCREMENT;
//                robotLift.setTargetPosition(targetTicks);
//                robotLift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//                robotLift.setPower(1);
//            }
//        }
//
//        dashboardTelemetry.addData("robot lift current ticks", robotLift.getCurrentPosition());
//        dashboardTelemetry.addData("lift lift current ticks", liftArm.getCurrentPosition());
//        dashboardTelemetry.addData("danger timer", dangerTimer.time());
//        dashboardTelemetry.update();
//    }
//
//    public void init() {
//        liftArm = opMode.hardwareMap.get(DcMotorEx.class, "liftArm");
//        robotLift = opMode.hardwareMap.get(DcMotorEx.class, "robotLift");
//
//        launch = new MM_Launch(opMode);
//
//        liftArm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        liftArm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//
//        robotLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        robotLift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//    }
//}
