package org.firstinspires.ftc.mmcenterstage;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue: Left", group = "Blue")
public class BlueLeft extends MM_OpMode {
    @Override
    public void runProcedures() {
        alliance = BLUE;
        int propPos = robot.drivetrain.purplePixelLeft();

            robot.transport.runToScorePos();
            robot.collector.deposit();
            robot.transport.goHome();


        if (propPos == 0){ //TODO determine parking area
            robot.drivetrain.strafeInches(21.5, .3);
        } else if (propPos == 1){
            robot.drivetrain.strafeInches(26, .4);
        } else {
            robot.drivetrain.strafeInches(31, .6);
        }
    }
}