package org.firstinspires.ftc.mmcenterstage;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue: Right", group = "Blue")

public class BlueRight extends MM_OpMode {
    @Override
    public void runProcedures() {
        alliance = BLUE;
        int propPos = robot.drivetrain.purplePixelRight();

        if (propPos == 1) {
            robot.drivetrain.strafeInches(-11.5, .3);
            robot.drivetrain.rotateToAngle(-45);
            robot.drivetrain.driveToAprilTag(10, 8, 27);
            robot.drivetrain.rotateToAngle(-90);
        } else if(propPos == 0) {

        } else {

        }
    }
}