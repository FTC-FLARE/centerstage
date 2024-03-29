package org.firstinspires.ftc.mmcenterstage;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;

@Autonomous(name = "Red: Right", group = "Red")
public class RedRight extends MM_Autos {
    @Override
    public void runProcedures() {
        alliance = RED;
        leftOrRight = RIGHT;
        startingPos = Math.abs(alliance + leftOrRight) - 1;

        int propPos = robot.drivetrain.purplePixel();
        robot.drivetrain.visionPortal.exposure.setMode(ExposureControl.Mode.Manual);
        //robot.drivetrain.visionPortal.gain.setGain(11);

        robot.drivetrain.rotateToAngle(85 * leftOrRight);

        int targetX = propPos == 2 ? -1 : 1;
        int tagToFind = alliance == BLUE ? propPos + 1 : propPos + 4;
        foundApriltagScoreYellow = robot.drivetrain.driveToAprilTag(tagToFind, targetX);

        if (foundApriltagScoreYellow) {
            robot.autoScoreOnBackDrop();
        }

        robot.drivetrain.park(propPos);
    }
}

