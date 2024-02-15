package org.firstinspires.ftc.mmcenterstage;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue: Left", group = "Blue")
public class BlueLeft extends MM_OpMode {
    @Override
    public void runProcedures() {
        alliance = BLUE;
        leftOrRight = LEFT;
        startingPos = Math.abs(alliance + leftOrRight) - 1 ;

        int propPos = robot.drivetrain.purplePixelLeft();
        int targetX = propPos == 2? -1: 1;
        int tagToFind = alliance == BLUE? propPos + 1: propPos + 4;

        robot.drivetrain.rotateToAngle(85 * alliance);
        MM_OpMode.foundApriltagScoreYellow = robot.drivetrain.driveToAprilTag(tagToFind, targetX);

        if (MM_OpMode.foundApriltagScoreYellow) {
            robot.transport.runToScorePos();
            robot.collector.deposit();
            robot.transport.goHome();
            robot.drivetrain.driveInches(2, .4);

            if (propPos == 0) { //TODO determine parking area
                robot.drivetrain.strafeInches(-31, .3);
            } else if (propPos == 1) {
                robot.drivetrain.strafeInches(24, .4);
            } else {
                robot.drivetrain.strafeInches(31, .3);
            }
            robot.drivetrain.driveInches(-12, .5);
        }else {
            robot.drivetrain.strafeInches(24, .4);
        }
    }
}