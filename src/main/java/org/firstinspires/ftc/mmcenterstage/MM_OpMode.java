package org.firstinspires.ftc.mmcenterstage;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class MM_OpMode extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    public static ElapsedTime matchTimer = new ElapsedTime();

    public static Gamepad currentGamepad1 = new Gamepad();
    public static Gamepad previousGamepad1 = new Gamepad();
    public static Gamepad currentGamepad2 = new Gamepad();
    public static Gamepad previousGamepad2 = new Gamepad();

    public MM_Robot robot = new MM_Robot(this);

    public static final int LEFT = 0;
    public static final int RIGHT = 1;
    public static int startingPos = 0;

    public static final int RED = 0;
    public static final int BLUE = 1;
    public static int alliance = 0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing... Please Wait");
        telemetry.update();
        robot.init();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        initProcedures();
        matchTimer.reset();
        runProcedures();
    }

    public void initProcedures(){
        waitForStart();
    }
    public abstract void runProcedures();
}

