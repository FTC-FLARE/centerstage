package org.firstinspires.ftc.mmcenterstage;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;

@Autonomous(name = "Blue: Left", group = "Blue")
public class BlueLeft extends MM_Autos {
    @Override
    public void runProcedures() {
        alliance = BLUE;
        leftOrRight = LEFT;
        startingPos = Math.abs(alliance + leftOrRight) - 1;

        int propPos = robot.drivetrain.purplePixel();
        int targetX = propPos == 2 ? -1 : 1;
        int tagToFind = alliance == BLUE ? propPos + 1 : propPos + 4;

        robot.drivetrain.rotateToAngle(85 * leftOrRight);
        robot.drivetrain.visionPortal.exposure.setMode(ExposureControl.Mode.Manual);

        foundApriltagScoreYellow = robot.drivetrain.driveToAprilTag(tagToFind, targetX);

        if (foundApriltagScoreYellow) {
            robot.autoScoreOnBackDrop();
        }

        robot.drivetrain.park(propPos);
    }
}