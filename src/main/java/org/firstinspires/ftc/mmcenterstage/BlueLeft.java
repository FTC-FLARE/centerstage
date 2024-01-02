package org.firstinspires.ftc.mmcenterstage;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

@Autonomous(name = "Blue: Left", group = "Blue")

public class BlueLeft extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    public Gamepad currentGamepad1 = new Gamepad();
    public Gamepad previousGamepad1 = new Gamepad();
    public Gamepad currentGamepad2Collect = new Gamepad();
    public Gamepad previousGamepad2Collect = new Gamepad();
    public MM_Robot robot = new MM_Robot(this, currentGamepad1, previousGamepad1, currentGamepad2Collect, previousGamepad2Collect, multipleTelemetry, true);

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing... Please Wait");
        telemetry.update();
        robot.init();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        while (opModeInInit()) {

        }

        waitForStart();

        int propPos = robot.drivetrain.purplePixelLeft(true);
        robot.transport.runToScorePos();
        robot.collector.score();
        robot.transport.goHome();
        if (propPos == 0){
            robot.drivetrain.strafeInches(21.5, .3);
        } else if (propPos == 1){
            robot.drivetrain.strafeInches(26, .4);
        } else {
            robot.drivetrain.strafeInches(31, .6);
        }

    }
}

