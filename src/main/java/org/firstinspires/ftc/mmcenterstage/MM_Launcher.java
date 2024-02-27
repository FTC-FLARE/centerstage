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

    public MM_Launcher(MM_OpMode opMode) {
        this.opMode = opMode;
        init();
    }

    public void unlock(){
        droneLock.setPosition(.5);
    }

    public void lift(){
        lift.setPosition(LAUNCH_POS);
    }

    public void disengageLift(){
        ServoControllerEx controller = (ServoControllerEx) lift.getController();
        controller.setServoPwmDisable(lift.getPortNumber());
    }

    public void launchDrone(){
        if (MM_TeleOp.currentGamepad1.left_bumper && !MM_TeleOp.previousGamepad1.left_bumper) {
            droneRelease.setPosition(1);
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
