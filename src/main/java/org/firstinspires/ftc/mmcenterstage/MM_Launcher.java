package org.firstinspires.ftc.mmcenterstage;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.util.ElapsedTime;
@Config
public class MM_Launcher {
    MM_OpMode opMode;

    private Servo droneRelease = null;
    private Servo lift = null;
    private Servo droneLock = null;

    public static double LAUNCH_POS = .448;

    ElapsedTime timeSinceLiftStart = new ElapsedTime();
    ElapsedTime lockTime = new ElapsedTime();


    private boolean inLaunchPosition = false;
    private boolean liftRequested = false;
    private boolean unlocked = false;

    public MM_Launcher(MM_OpMode opMode) {
        this.opMode = opMode;
        init();
    }

    public void launchControl(){
        if (!liftRequested) {
            if (MM_TeleOp.currentGamepad1.dpad_up && !MM_TeleOp.previousGamepad1.dpad_up){ //&& MM_OpMode.matchTimer.time() > 87) { TODO add match timer back
                droneLock.setPosition(0);
                lockTime.reset();
                liftRequested = true;
            }
        } else {
            if (!unlocked) {
                if (lockTime.milliseconds() > 200) {
                    lift.setPosition(LAUNCH_POS);
                    unlocked = true;
                }
            } else { // unlocked`
                if (!inLaunchPosition && lockTime.milliseconds() >= 1200) {
                    ServoControllerEx controller = (ServoControllerEx) lift.getController();
                    controller.setServoPwmDisable(lift.getPortNumber());

                    inLaunchPosition = true;
                }

                if (inLaunchPosition && (MM_TeleOp.currentGamepad1.dpad_down && !MM_TeleOp.previousGamepad1.dpad_down)) {
                    droneRelease.setPosition(1);
                }
            }
        }
    }

    public void init() {
        droneRelease = opMode.hardwareMap.get(Servo.class, "droneRelease");
        droneRelease.setPosition(0);

        droneLock = opMode.hardwareMap.get(Servo.class, "droneLock");
        droneLock.setPosition(1);

        lift = opMode.hardwareMap.get(Servo.class, "droneLift");
        lift.setPosition(0);
    }
}
