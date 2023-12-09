package org.firstinspires.ftc.mmcenterstage;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@ TeleOp(name = "Tele-op", group = "mm")

public class MM_TeleOp extends LinearOpMode {

    public static Gamepad currentGamepad1 = new Gamepad();
    public static Gamepad previousGamepad1 = new Gamepad();
    public static Gamepad currentGamepad2 = new Gamepad();
    public static Gamepad previousGamepad2 = new Gamepad();
    public static ElapsedTime matchTimer = new ElapsedTime();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    public MM_Robot robot = new MM_Robot(this, currentGamepad1, previousGamepad1, currentGamepad2,
            previousGamepad2, dashboardTelemetry);
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing... Please Wait");
        telemetry.update();
        robot.init();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        matchTimer.reset();

        while(opModeIsActive()){
            robot.drivetrain.driveWithSticks();
            robot.collector.collect();
            robot.transport.transport();
            robot.liftLift.liftLift();
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);

            telemetry.update();
            dashboardTelemetry.update();
        }
        robot.drivetrain.aprilTags.visionPortal.close();
    }
}

