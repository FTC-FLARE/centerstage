//package org.firstinspires.ftc.mmcenterstage;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.Servo;
//
//public class MM_Launch {
//    LinearOpMode opMode;
//
//    private Servo droneReleaser = null;
//
//    public MM_Launch(LinearOpMode opMode) {
//        this.opMode = opMode;
//    }
//
//    public void launch() {
//        if (MM_TeleOp.matchTimer.time() > 90 && MM_LiftLift.isInLaunchPos) {
//            if (opMode.gamepad2.right_trigger > 0.1) {
//                droneReleaser.setPosition(1);
//            }
//        }
//    }
//
//    public void init() {
//        droneReleaser = opMode.hardwareMap.get(Servo.class, "droneReleaser");
//
//        droneReleaser.setPosition(0);
//    }
//}
