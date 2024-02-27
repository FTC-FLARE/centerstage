package org.firstinspires.ftc.mmcenterstage;

import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MM_Robot {
    private final MM_OpMode opMode;
    public MM_Drivetrain drivetrain;
    public MM_Collector collector;
    public MM_Transport transport;
    public MM_Lift lift;
    public MM_Launcher launcher;

    public static int endGameStatus = 0;
    ElapsedTime endGameTimer = new ElapsedTime();


    public MM_Robot(MM_OpMode opMode) {
        this.opMode = opMode;
        endGameStatus = 0;
    }

    public void init() {
        drivetrain = new MM_Drivetrain(opMode);
        collector = new MM_Collector(opMode);
        transport = new MM_Transport(opMode);
        lift = new MM_Lift(opMode);
        launcher = new MM_Launcher(opMode);
    }

    public void autoScoreOnBackDrop() {
        transport.runToScorePos();
        collector.deposit();
//        transport.goHome();
        drivetrain.driveInchesAndLowerSlide(2, .4);
    }

    public void endGameControl() {
        switch (endGameStatus) {
            case 0: // not started
                if (MM_TeleOp.currentGamepad1.right_bumper && !MM_TeleOp.previousGamepad1.right_bumper && MM_OpMode.matchTimer.time() > 87) {
                    launcher.unlock();
                    endGameTimer.reset();
                    endGameStatus = 1;
                }
                break;
            case 1: // wait to finish unlocking launcher
                if (endGameTimer.milliseconds() > 200) {
                    launcher.lift();
                    endGameStatus = 2;
                }
                break;
            case 2: // wait to finish lifting launcher
                if (endGameTimer.milliseconds() >= 1200) {
                    launcher.disengageLift();
                    lift.release();
                    endGameStatus = 3;
                }
                break;
            case 3: //ready to operate
                lift.control();
                launcher.launchDrone();
        }
    }
}
