package org.firstinspires.ftc.mmcenterstage;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Tele-op", group = "mm")

public class MM_TeleOp extends MM_OpMode {
    @Override
    public void runProcedures() {
        while(opModeIsActive()){
            robot.drivetrain.driveWithSticks();
            robot.drivetrain.kickPixels();
            robot.collector.control();
            robot.transport.control();
            robot.endGameControl();

            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);

            multipleTelemetry.addData("Distance", robot.drivetrain.getDistance(1));
            multipleTelemetry.update();
        }
    }
}