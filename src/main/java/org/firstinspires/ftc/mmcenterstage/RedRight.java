package org.firstinspires.ftc.mmcenterstage;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red: Right", group = "Red")

public class RedRight extends MM_OpMode {
    @Override
    public void runProcedures() {
        setIsAuto(true);
        int propPos = robot.drivetrain.purplePixelRight(false);
        robot.transport.runToScorePos();
        robot.collector.score();
        robot.transport.goHome();
        if (propPos == 4) {
            robot.drivetrain.strafeInches(-31, .3);
        } else if (propPos == 5) {
            robot.drivetrain.strafeInches(-26, .4);
        } else {
            robot.drivetrain.strafeInches(-21.5, .6);
        }
    }
}

