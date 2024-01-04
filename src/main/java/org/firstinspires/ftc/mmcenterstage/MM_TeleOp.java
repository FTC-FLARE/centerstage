package org.firstinspires.ftc.mmcenterstage;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Tele-op", group = "mm")

public class MM_TeleOp extends MM_OpMode {
    @Override
    public void runProcedures() {
        while(opModeIsActive()){
            robot.drivetrain.driveWithSticks();
            robot.collector.control();
            robot.transport.transport();
            robot.liftLift.liftLift();

            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);

            telemetry.update();
            multipleTelemetry.update();
        }
    }
}