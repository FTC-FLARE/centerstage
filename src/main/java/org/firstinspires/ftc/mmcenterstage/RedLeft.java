package org.firstinspires.ftc.mmcenterstage;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red: Left", group = "Red")

public class RedLeft extends MM_OpMode {
    @Override
    public void runProcedures() {
        alliance = RED;
        leftOrRight = LEFT;
        startingPos = Math.abs(alliance + leftOrRight) - 1;
        int propPos = robot.drivetrain.purplePixelLeft();

        if (propPos == 1) {
            robot.drivetrain.strafeInches(-19, .3);
            robot.drivetrain.rotateToAngle(-90);
            robot.drivetrain.driveInches(-76, .5);
            robot.drivetrain.strafeInches(22, .5);
            robot.drivetrain.driveToAprilTag(2, 0, 6.8, 0);

            robot.transport.runToScorePos();
            robot.collector.deposit();
            robot.transport.goHome();

        } else if (propPos == 0) {
            robot.drivetrain.strafeInches(-9.5, .5);
            robot.drivetrain.driveInches(-76, .5);
            robot.drivetrain.strafeInches(22, .5);
            robot.drivetrain.driveToAprilTag(1, 0, 6.8, 0);

            robot.transport.runToScorePos();
            robot.collector.deposit();
            robot.transport.goHome();
        } else {

        }

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