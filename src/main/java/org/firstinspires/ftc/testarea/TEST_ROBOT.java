package org.firstinspires.ftc.testarea;

import android.graphics.LinearGradient;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.mmcenterstage.MM_Collector;
import org.firstinspires.ftc.mmcenterstage.MM_Drivetrain;
import org.firstinspires.ftc.mmcenterstage.MM_Launcher;
import org.firstinspires.ftc.mmcenterstage.MM_Lift;
import org.firstinspires.ftc.mmcenterstage.MM_OpMode;
import org.firstinspires.ftc.mmcenterstage.MM_Transport;

public class TEST_ROBOT {
    private final LinearOpMode opMode;
    public TEST_DRIVETRAIN drivetrain;

    public TEST_ROBOT(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void init() {
        drivetrain = new TEST_DRIVETRAIN(opMode);
    }
}