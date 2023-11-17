package org.firstinspires.ftc.mmcenterstage;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MM_Robot {
    private final LinearOpMode opMode;
    private final Gamepad currentGamepad1;
    private final Gamepad previousGamepad1;
    private final Gamepad currentGamepad2Collect;
    private final Gamepad previousGamepad2Collect;
    private final Telemetry dashboardTelemetry;

    public MM_Drivetrain drivetrain;
    public MM_Collector collector;
    public MM_Transport transport;

    public MM_Robot(LinearOpMode opMode, Gamepad currentGamepad1, Gamepad previousGamepad1,
                    Gamepad currentGamepad2Collect, Gamepad previousGamepad2Collect, Telemetry dashboardTelemetry) {
        this.opMode = opMode;
        this.currentGamepad1 = currentGamepad1;
        this.previousGamepad1 = previousGamepad1;
        this.currentGamepad2Collect = currentGamepad2Collect;
        this.previousGamepad2Collect = previousGamepad2Collect;
        this.dashboardTelemetry = dashboardTelemetry;

    }

    public void init() {
        drivetrain = new MM_Drivetrain(opMode, currentGamepad1, previousGamepad1, dashboardTelemetry);
        collector = new MM_Collector(opMode, currentGamepad2Collect, previousGamepad2Collect);
        transport = new MM_Transport(opMode, dashboardTelemetry);
    }

}
