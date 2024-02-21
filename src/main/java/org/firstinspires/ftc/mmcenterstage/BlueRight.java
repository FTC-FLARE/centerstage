package org.firstinspires.ftc.mmcenterstage;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;

@Autonomous(name = "Blue: Right", group = "Blue")

public class BlueRight extends MM_Autos {
    @Override
    public void runProcedures() {
        alliance = BLUE;
        leftOrRight = RIGHT;
        startingPos = Math.abs(alliance + leftOrRight) - 1;

        int propPos = robot.drivetrain.purplePixel();
        int targetX = propPos == 2 ? -1 : 1;
        int tagToFindOnBackdrop = alliance == BLUE ? propPos + 1 : propPos + 4;
        int tagToFindOnWall = alliance == BLUE ? 10 : 8;


        robot.drivetrain.rotateToAngle(-85 * leftOrRight);
        robot.drivetrain.visionPortal.exposure.setMode(ExposureControl.Mode.Manual);

        robot.drivetrain.driveToAprilTag(tagToFindOnWall, -9.25, 25.3, 0);

//        if (propPos == 1) {
//            robot.drivetrain.strafeInches(19, .3);
//            robot.drivetrain.rotateToAngle(90);
//            robot.drivetrain.driveInches(-76, .5);
//            robot.drivetrain.strafeInches(-22, .5);
//            robot.drivetrain.driveToAprilTag(2, 0); // targetY was 3.4
//
//            if (foundApriltagScoreYellow) {
//                robot.autoScoreOnBackDrop();
//            }
//
//        } else if (propPos == 0) {
//            robot.drivetrain.strafeInches(17.7, .5);
//            robot.drivetrain.driveInches(-76, .5);
//            robot.drivetrain.strafeInches(-22, .5);
//            foundApriltagScoreYellow = robot.drivetrain.driveToAprilTag(1, 0); // targetY was 3.4
//
//            if (foundApriltagScoreYellow) {
//                robot.autoScoreOnBackDrop();
//            }
//        } else {
//
//        }
    }
}