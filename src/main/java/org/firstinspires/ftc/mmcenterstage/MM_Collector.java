package org.firstinspires.ftc.mmcenterstage;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
public class MM_Collector {
    private final MM_OpMode opMode;

    private DcMotorEx leftWheel = null;
    private CRServo innertakeStar = null;

    public static double RIGHT_WHEEL_POWER = 1;
    public static double LEFT_WHEEL_POWER = 1;
    public static double INNERTAKE_STAR_POWER = 1;

    public MM_Collector(MM_OpMode opMode) {
        this.opMode = opMode;
        init();
    }

    public void control() {
        if (opMode.gamepad2.left_bumper) {
            leftWheel.setPower(LEFT_WHEEL_POWER);
            innertakeStar.setPower(INNERTAKE_STAR_POWER);
        } else if (opMode.gamepad2.left_trigger > 0.1) {
            leftWheel.setPower(-LEFT_WHEEL_POWER);
            innertakeStar.setPower(opMode.robot.transport.atBottom() ? -INNERTAKE_STAR_POWER : -.2);
        } else {
            leftWheel.setPower(0);


            innertakeStar.setPower(0);
        }
    }

    public void deposit() {
        innertakeStar.setPower(-.2);
        opMode.sleep(2000);
        innertakeStar.setPower(0);
    }

    public void init() {
        leftWheel = opMode.hardwareMap.get(DcMotorEx.class, "leftWheel");
        innertakeStar = opMode.hardwareMap.get(CRServo.class, "innerWheel");

        leftWheel.setDirection(DcMotorEx.Direction.REVERSE);
    }
}