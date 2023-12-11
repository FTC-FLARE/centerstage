package org.firstinspires.ftc.mmcenterstage;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name = "MM Auto", group = "mm")

public class MM_Autos extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    public Gamepad currentGamepad1 = new Gamepad();
    public Gamepad previousGamepad1 = new Gamepad();
    public Gamepad currentGamepad2Collect = new Gamepad();
    public Gamepad previousGamepad2Collect =  new Gamepad();
    public MM_Robot robot = new MM_Robot(this, currentGamepad1, previousGamepad1, currentGamepad2Collect, previousGamepad2Collect, dashboardTelemetry);

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing... Please Wait");
        telemetry.update();
        robot.init();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        while (opModeInInit()) {
//            robot.drivetrain.getAprilTagId(1);
//            robot.drivetrain.getTfodId();
//            dashboardTelemetry.update();
        }

        waitForStart();

        //robot.drivetrain.driveToAprilTag();
        robot.drivetrain.rotateToAngle(90);

        //robot.drivetrain.aprilTags.visionPortal.close();
    }
}

