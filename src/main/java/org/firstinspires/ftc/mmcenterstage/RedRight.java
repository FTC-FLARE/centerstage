package org.firstinspires.ftc.mmcenterstage;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red: Right", group = "Red")
public class RedRight extends MM_OpMode {
    @Override
    public void runProcedures() {
        alliance = RED;
        leftOrRight = RIGHT;
        startingPos = Math.abs(alliance + leftOrRight) - 1;

        int propPos = robot.drivetrain.purplePixel();
        robot.drivetrain.rotateToAngle(85 * alliance);

        int targetX = propPos == 2 ? -1 : 1;
        int tagToFind = alliance == BLUE ? propPos + 1 : propPos + 4;
        MM_OpMode.foundApriltagScoreYellow = robot.drivetrain.driveToAprilTag(tagToFind, targetX);

        if (MM_OpMode.foundApriltagScoreYellow) {
            robot.autoScoreOnBackDrop();
        }

        robot.drivetrain.park(propPos);
    }
}

