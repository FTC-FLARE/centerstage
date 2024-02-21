package org.firstinspires.ftc.mmcenterstage;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;

@Autonomous(name = "Autos", group = "MM")
public class MM_Autos extends MM_OpMode {

    public static final int LEFT = -1;
    public static final int RIGHT = 1;
    public static int leftOrRight = RIGHT;
    public static final int RED = -1;
    public static final int BLUE = 1;
    public static int alliance = BLUE;
    public static final int BACKDROP = -1;
    public static final int AUDIENCE = 1;

    public static final int MIDDLE = -1;
    public static final int CORNER = 1;
    public static int startingPos = BACKDROP;
    public static boolean foundApriltagScoreYellow = false;
    public static boolean colorChosen = false;

    public static boolean locationChosen = false;

    public static int finalPark;
    public static boolean parkChosen = false;

    public static int propPos;

    private static int targetX;
    private static int tagToFindOnBackdrop;
    private static int tagToFindOnWall;

    //@Override
    //public void initProcedures() {
        //waitForStart();
//        int[] colorList = new int[2];
//        colorList[0] = BLUE;
//        colorList[1] = RED;
//
//        int currentColor = colorList[0];
//
//        int[] locationList = new int[2];
//        locationList[0] = BACKDROP;
//        locationList[1] = AUDIENCE;
//
//        int currentLocation = locationList[0];
//
//        int[] parkingList = new int[2];
//
//        parkingList[0] = MIDDLE;
//        parkingList[1] = CORNER;
//
//        int currentPark = parkingList[0];
//
//        while (opModeInInit()) {
//            if (!colorChosen) {
//                telemetry.addData("color", currentColor);
//                telemetry.update();
//
//                if ((currentGamepad1.dpad_up && !previousGamepad1.dpad_up) || currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
//                    currentColor = currentColor == colorList[1] ? colorList[0] : colorList[1];
//                }
//                if (currentGamepad1.a && !previousGamepad1.a) {
//                    colorChosen = true;
//                    alliance = currentColor;
//                }
//            } else if (!locationChosen) {
//                telemetry.addData("location", currentLocation);
//                telemetry.update();
//
//                if ((currentGamepad1.dpad_up && !previousGamepad1.dpad_up) || currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
//                    currentLocation = currentLocation == locationList[1] ? locationList[0] : locationList[1];
//                }
//                if (currentGamepad1.a && !previousGamepad1.a) {
//                    locationChosen = true;
//                    startingPos = currentLocation;
//                }
//            } else {
//                telemetry.addData("parking location", currentPark);
//                telemetry.update();
//
//                if ((currentGamepad1.dpad_up && !previousGamepad1.dpad_up) || currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
//                    currentPark = currentPark == parkingList[1] ? parkingList[0] : parkingList[1];
//                }
//                if (currentGamepad1.a && !previousGamepad1.a) {
//                    parkChosen = true;
//                    finalPark = currentPark;
//                }
//            }
//        }

    //}

    @Override
    public void runProcedures() {
        startingPos = Math.abs(alliance + leftOrRight) - 1;

        propPos = robot.drivetrain.purplePixel();

        prepareForAprilTag();

        if (startingPos == AUDIENCE){
            robot.drivetrain.driveToAprilTag(tagToFindOnWall, -9.25, 25.3, 0);
        } else {
            foundApriltagScoreYellow = robot.drivetrain.driveToAprilTag(targetX, tagToFindOnBackdrop);
            if (foundApriltagScoreYellow){
                robot.autoScoreOnBackDrop();
            }
            robot.drivetrain.park(propPos);
        }
    }

    public void prepareForAprilTag() {
        targetX = propPos == 2 ? -1 : 1;
        tagToFindOnBackdrop = alliance == BLUE ? propPos + 1 : propPos + 4;
        tagToFindOnWall = alliance == BLUE ? 10 : 8;

        robot.drivetrain.rotateToAngle(-85 * leftOrRight);
        robot.drivetrain.visionPortal.exposure.setMode(ExposureControl.Mode.Manual);
    }


}