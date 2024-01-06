package org.firstinspires.ftc.mmcenterstage;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red: Left", group = "Red")

public class RedLeft extends MM_OpMode {
    @Override
    public void runProcedures() {
        alliance = RED;
        robot.drivetrain.purplePixelLeft();
    }

    @Override
    public void initProcedures() {
        while (opModeInInit()) {
//            robot.drivetrain.getAprilTagId(1);
//            robot.drivetrain.getTfodId();
//            dashboardTelemetry.update();
            telemetry.addData("name", getClass().getSimpleName());
            telemetry.update();
        }
    }
}