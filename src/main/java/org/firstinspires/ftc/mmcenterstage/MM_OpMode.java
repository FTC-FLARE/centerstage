package org.firstinspires.ftc.mmcenterstage;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.util.List;

import java.sql.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.ListIterator;

public abstract class MM_OpMode extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    public static ElapsedTime matchTimer = new ElapsedTime();

    public static Gamepad currentGamepad1 = new Gamepad();
    public static Gamepad previousGamepad1 = new Gamepad();
    public static Gamepad currentGamepad2 = new Gamepad();
    public static Gamepad previousGamepad2 = new Gamepad();

    public MM_Robot robot = new MM_Robot(this);

    public static final int LEFT = -1;
    public static final int RIGHT = 1;
    public static int leftOrRight = LEFT;
    public static final int RED = -1;
    public static final int BLUE = 1;
    public static int alliance = RED;
    public static final int BACKDROP = -1;
    public static final int AUDIENCE = 1;

    public static final int MIDDLE = -1;
    public static final int CORNER = 1;
    public static int startingPos = BACKDROP;
    public static boolean foundApriltagScoreYellow = false;
    public static int finalColor;
    public static boolean colorChosen = false;

    public static int finalLocation;
    public static boolean locationChosen = false;

    public static int finalPark;
    public static boolean parkChosen = false;



    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing... Please Wait");
        telemetry.update();
        robot.init();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        if (!getClass().getSimpleName().equals("MM_TeleOp") ) {
            initProcedures(true);
        } else {
            initProcedures();
        }

        matchTimer.reset();
        runProcedures();
    }

    public void initProcedures(){
        waitForStart();
    }

    public void initProcedures(boolean initOptions){
        int[] colorList = new int[2];
        colorList[0] = BLUE;
        colorList[1] = RED;

        int currentColor = colorList[0];

        int[] locationList = new int[2];
        locationList[0] = BACKDROP;
        locationList[1] = AUDIENCE;

        int currentLocation = locationList[0];

        int[] parkingList = new int[2];

        parkingList[0] = MIDDLE;
       parkingList[1] = CORNER;

        int currentPark = parkingList[0];

        while (opModeInInit()){
            if (!colorChosen) {
                telemetry.addData("color", currentColor);
                telemetry.update();

                if ((currentGamepad1.dpad_up && !previousGamepad1.dpad_up) || currentGamepad1.dpad_down && !previousGamepad1.dpad_down){
                    currentColor = currentColor == colorList[1]? colorList[0]: colorList[1];
                }
                if(currentGamepad1.a && !previousGamepad1.a){
                    colorChosen = true;
                    alliance = currentColor;
                }
            } else if (!locationChosen){
                telemetry.addData("location", currentLocation);
                telemetry.update();

                if ((currentGamepad1.dpad_up && !previousGamepad1.dpad_up) || currentGamepad1.dpad_down && !previousGamepad1.dpad_down){
                    currentLocation = currentLocation == locationList[1]? locationList[0]: locationList[1];
                }
                if(currentGamepad1.a && !previousGamepad1.a){
                    locationChosen = true;
                    startingPos = currentLocation;
                }
            } else {
                telemetry.addData("parking location", currentPark);
                telemetry.update();

                if ((currentGamepad1.dpad_up && !previousGamepad1.dpad_up) || currentGamepad1.dpad_down && !previousGamepad1.dpad_down){
                    currentPark = currentPark == parkingList[1]? parkingList[0]: parkingList[1];
                }
                if(currentGamepad1.a && !previousGamepad1.a){
                    parkChosen = true;
                    finalPark = currentPark;
                }
            }

        }
    }
    public abstract void runProcedures();
}

