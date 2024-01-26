package org.firstinspires.ftc.mmcenterstage;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red: Right", group = "Red")
public class RedRight extends MM_OpMode {
    @Override
    public void runProcedures() {
        alliance = RED;
        int propPos = robot.drivetrain.purplePixelRight();

        if (MM_OpMode.foundApriltagScoreYellow) {
            robot.transport.runToScorePos();
            robot.collector.deposit();
            robot.transport.goHome();
            robot.drivetrain.driveInches(2, .4);

            if (propPos == 0) {
                robot.drivetrain.strafeInches(-31, .3);
            } else if (propPos == 1) {
                robot.drivetrain.strafeInches(-24, .4);
            } else {
                robot.drivetrain.strafeInches(31, .3);
            }
        }else {
//            robot.drivetrain.strafeInches(-24, .4);
        }
    }
}

