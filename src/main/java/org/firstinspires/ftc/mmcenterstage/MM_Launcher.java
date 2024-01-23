package org.firstinspires.ftc.mmcenterstage;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MM_Launcher {
    MM_OpMode opMode;

    private Servo droneRelease = null;
    private Servo lift = null;
    ElapsedTime liftTime = new ElapsedTime();

    private boolean inLaunchPosition = false;
    private boolean liftRequested = false;

    public MM_Launcher(MM_OpMode opMode) {
        this.opMode = opMode;
        init();
    }

    public void launchControl(){
        if (MM_TeleOp.currentGamepad1.dpad_up && !MM_TeleOp.previousGamepad1.dpad_up){
            lift.setPosition(.45);
            liftTime.reset();
            liftRequested = true;
        }
        if (liftRequested && !inLaunchPosition && liftTime.time() >= .75){
            ServoControllerEx controller = (ServoControllerEx) lift.getController();
            controller.setServoPwmDisable(lift.getPortNumber());

            inLaunchPosition = true;
        }
        if (inLaunchPosition && (MM_TeleOp.currentGamepad1.dpad_down && !MM_TeleOp.previousGamepad1.dpad_down)){
            droneRelease.setPosition(1);
        }
    }

    public void init() {
        droneRelease = opMode.hardwareMap.get(Servo.class, "droneRelease");
        droneRelease.setPosition(0);

        lift = opMode.hardwareMap.get(Servo.class, "droneLift");
        lift.setPosition(0);
    }
}
