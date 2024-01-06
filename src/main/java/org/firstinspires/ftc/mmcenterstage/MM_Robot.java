package org.firstinspires.ftc.mmcenterstage;

public class MM_Robot {
    private final MM_OpMode opMode;
    public MM_Drivetrain drivetrain;
    public MM_Collector collector;
    public MM_Transport transport;
    public MM_Lift liftLift;

    public MM_Robot(MM_OpMode opMode) {
        this.opMode = opMode;
    }

    public void init() {
        drivetrain = new MM_Drivetrain(opMode);
        collector = new MM_Collector(opMode);
        transport = new MM_Transport(opMode);
        liftLift = new MM_Lift(opMode);
    }
}