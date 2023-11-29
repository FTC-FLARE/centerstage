package org.firstinspires.ftc.mmcenterstage;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MM_Robot {
    private final LinearOpMode opMode;
    public final Gamepad currentGamepad1;
    public final Gamepad previousGamepad1;
    private final Gamepad currentGamepad2;
    private final Gamepad previousGamepad2;
    private final Telemetry dashboardTelemetry;

    public MM_Drivetrain drivetrain;
//    public MM_Collector collector;
//    public MM_Transport transport;
    //public MM_LiftLift liftLift;

    public MM_Robot(LinearOpMode opMode, Gamepad currentGamepad1, Gamepad previousGamepad1,
                    Gamepad currentGamepad2, Gamepad previousGamepad2, Telemetry dashboardTelemetry) {
        this.opMode = opMode;
        this.currentGamepad1 = currentGamepad1;
        this.previousGamepad1 = previousGamepad1;
        this.currentGamepad2 = currentGamepad2;
        this.previousGamepad2 = previousGamepad2;
        this.dashboardTelemetry = dashboardTelemetry;

    }

    public void init() {
        drivetrain = new MM_Drivetrain(opMode, dashboardTelemetry);
//        collector = new MM_Collector(opMode);
//        transport = new MM_Transport(opMode, dashboardTelemetry);
        //liftLift = new MM_LiftLift(opMode, dashboardTelemetry);
    }

}
