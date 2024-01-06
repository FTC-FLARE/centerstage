package org.firstinspires.ftc.mmcenterstage;

import com.qualcomm.robotcore.hardware.Servo;

public class MM_Launcher {
    MM_OpMode opMode;

    private Servo droneRelease = null;
    private Servo lift = null;

    public MM_Launcher(MM_OpMode opMode) {
        this.opMode = opMode;
    }
//
//    public void launch() {
//        if (MM_TeleOp.matchTimer.time() > 90 && MM_LiftLift.isInLaunchPos) {
//            if (opMode.gamepad2.right_trigger > 0.1) {
//                droneReleaser.setPosition(1);
//            }
//        }
//    }
//
    public void init() {
        droneRelease = opMode.hardwareMap.get(Servo.class, "droneRelease");
        droneRelease.setPosition(0);

        lift = opMode.hardwareMap.get(Servo.class, "droneLift");
    }
}
