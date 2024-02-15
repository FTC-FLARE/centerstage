package org.firstinspires.ftc.mmcenterstage;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue: Right", group = "Blue")

public class BlueRight extends MM_OpMode {
    @Override
    public void runProcedures() {
        alliance = BLUE;
        leftOrRight = RIGHT;
        startingPos = Math.abs(alliance + leftOrRight) - 1 ;

        int propPos = robot.drivetrain.purplePixelRight();
        int targetX = propPos == 2? -1: 1;
        int tagToFind = alliance == BLUE? propPos + 1: propPos + 4;

        robot.drivetrain.rotateToAngle(85 * alliance);
        MM_OpMode.foundApriltagScoreYellow = robot.drivetrain.driveToAprilTag(tagToFind, targetX);

        if (propPos == 1) {
            robot.drivetrain.strafeInches(19, .3);
            robot.drivetrain.rotateToAngle(90);
            robot.drivetrain.driveInches(-76, .5);
            robot.drivetrain.strafeInches(-22, .5);
            robot.drivetrain.driveToAprilTag(2, 0, 3.4, 0);

            robot.transport.runToScorePos();
            robot.collector.deposit();
            robot.transport.goHome();

        } else if (propPos == 0) {
            robot.drivetrain.strafeInches(17.7, .5);
            robot.drivetrain.driveInches(-76, .5);
            robot.drivetrain.strafeInches(-22, .5);
            robot.drivetrain.driveToAprilTag(1, 0, 3.4, 0);

            robot.transport.runToScorePos();
            robot.collector.deposit();
            robot.transport.goHome();
        } else {
        }
    }
}