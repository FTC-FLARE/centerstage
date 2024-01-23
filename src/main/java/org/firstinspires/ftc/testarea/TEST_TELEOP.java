package org.firstinspires.ftc.testarea;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.mmcenterstage.MM_OpMode;

@TeleOp(name = "Test BOB", group = "mm")

public class TEST_TELEOP extends LinearOpMode {

    public TEST_ROBOT robot = new TEST_ROBOT(this);

    @Override
    public void runOpMode() {
        robot.init();
        waitForStart();



//        MM_OpMode.alliance = MM_OpMode.BLUE;
//        int propPos = robot.drivetrain.purplePixelLeft();
//
//        if (propPos == 0){ //TODO determine parking area
//            robot.drivetrain.strafeInches(-31.5, .3);
//        } else if (propPos == 1){
//            robot.drivetrain.strafeInches(25, .35);
//        } else {
//            robot.drivetrain.strafeInches(31, .4);
//        }
//

//        while(opModeIsActive()){
//            robot.drivetrain.driveWithSticks();
//            robot.drivetrain.kickPixels();
//        }
    }
}