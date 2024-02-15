package org.firstinspires.ftc.mmcenterstage;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autos", group = "MM")
public class Autos extends MM_OpMode {
    @Override
    public void runProcedures() {

        startingPos = Math.abs(alliance + leftOrRight) - 1;

        int propPos = robot.drivetrain.purplePixel();
        int targetX = propPos == 2 ? -1 : 1;
        int tagToFind = alliance == BLUE ? propPos + 1 : propPos + 4;

        robot.drivetrain.rotateToAngle(85 * alliance);
        MM_OpMode.foundApriltagScoreYellow = robot.drivetrain.driveToAprilTag(tagToFind, targetX);

        if (MM_OpMode.foundApriltagScoreYellow) {
            robot.autoScoreOnBackDrop();
        }

        robot.drivetrain.park(propPos);
    }
}