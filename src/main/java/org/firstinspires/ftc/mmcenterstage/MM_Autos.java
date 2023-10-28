package org.firstinspires.ftc.mmcenterstage;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Auto BOB", group = "mm")

public class MM_Autos extends LinearOpMode {
    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    public Gamepad currentGamepad1 = new Gamepad();
    public Gamepad previousGamepad1 = new Gamepad();
    public MM_Robot robot = new MM_Robot(this, currentGamepad1, previousGamepad1, dashboardTelemetry);

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing... Please Wait");
        telemetry.update();
        robot.init();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        robot.drivetrain.driveToAprilTag();

        robot.drivetrain.aprilTags.visionPortal.close();
    }
}

