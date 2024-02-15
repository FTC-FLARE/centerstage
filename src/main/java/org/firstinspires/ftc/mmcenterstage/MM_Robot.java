package org.firstinspires.ftc.mmcenterstage;

public class MM_Robot {
    private final MM_OpMode opMode;
    public MM_Drivetrain drivetrain;
    public MM_Collector collector;
    public MM_Transport transport;
    public MM_Lift lift;
    public MM_Launcher launcher;

    public MM_Robot(MM_OpMode opMode) {
        this.opMode = opMode;
    }

    public void init() {
        drivetrain = new MM_Drivetrain(opMode);
        collector = new MM_Collector(opMode);
        transport = new MM_Transport(opMode);
        lift = new MM_Lift(opMode);
        launcher = new MM_Launcher(opMode);
    }

    public void autoScoreOnBackDrop(){
        transport.runToScorePos();
        collector.deposit();
        transport.goHome();
        drivetrain.driveInches(2, .4);
    }
}