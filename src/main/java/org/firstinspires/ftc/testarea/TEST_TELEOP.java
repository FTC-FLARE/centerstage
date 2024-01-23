package org.firstinspires.ftc.testarea;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.mmcenterstage.MM_OpMode;

@TeleOp(name = "Test BOB", group = "mm")

public class TEST_TELEOP extends LinearOpMode {

    public TEST_ROBOT Trobot = new TEST_ROBOT(this);

    @Override
    public void runOpMode() {
        Trobot.init();
        waitForStart();

        while(opModeIsActive()){
            Trobot.drivetrain.driveWithSticks();
            Trobot.drivetrain.kickPixels();
        }
    }
}