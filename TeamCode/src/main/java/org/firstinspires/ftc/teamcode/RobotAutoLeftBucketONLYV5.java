package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Topaz: Auto Drive Position Left (Bucket Only) V5", group="Robot")
public class RobotAutoLeftBucketONLYV5 extends AutoBase4 {

    @Override
    public void runOpMode() {
        initializeHardware(true);
        while (!isStopRequested() && robot.positionTracker.resetPositionCheck(0,240,0)) {
            telemetry.addData("POSITION - Current X",  "%.2f", robot.positionTracker.getXPositionCM());
            telemetry.addData("POSITION - Current Y",  "%.2f", robot.positionTracker.getYPositionCM());
            telemetry.addData("POSITION - Current heading", robot.positionTracker.getHeadingDegrees());
            telemetry.addData(">", "CHARLIE v5 - *NOT READY*.");
            telemetry.update();
            sleep(100);
        }
        telemetry.addData(">", "CHARLIE v5 *READY* - LEFT AUTO.  Press START.");    //
        telemetry.update();
        waitForStart();

        performActionsWithDelays("DROP OFF SAMPLE - STEP 1",
                moveExtensionEncoderAction(robot.getCurrentExtensionPosition(), 0), 300,
                moveArmEncoderAction(robot.getCurrentArmPosition(), robot.DRIVE_ARM_POSITION), 300,
                driveToPositionAction(robot.RELEASE_SAMPLE_POS_X, robot.RELEASE_SAMPLE_POS_Y, robot.RELEASE_SAMPLE_DEGREES, false), 0,
                null, 0, null);

        //step 6: drop off the sample arm position
        performActionsWithDelays("DROP OFF SAMPLE - STEP 2",
                moveArmEncoderAction(robot.getCurrentArmPosition(), robot.RELEASE_SAMPLE_ARM_HEIGHT), 0,
                moveExtensionEncoderAction(robot.getCurrentExtensionPosition(), robot.RELEASE_SAMPLE_EXTENSION_POSITION), 500,
                null, 0, null, 0, null);

        //step 6: drop off the sample by lowering arm
        performActionsWithDelays("DROP OFF SAMPLE - STEP 2",
                moveArmEncoderAction(robot.getCurrentArmPosition(), robot.RELEASE_SAMPLE_ARM_HEIGHT_2), 0,
                null, 0,
                null, 0, null, 0, null);

        //step 6: drop off the sample
        performActionsWithDelays("DROP OFF SAMPLE - STEP 3",
                moveIntakeTimedAction(false, true, 800, false, this), 900,
//                moveExtensionEncoderAction(robot.getCurrentExtensionPosition(), 0), 0,
                null, 0, null, 0, null,0,null);

        //step 7:pickup second block from the floor
        robot.resetDrivePosition();

//        // Step 1:  position to hook specimen
//        performActionsWithDelays("DRIVE TO POSITION",
//            driveToPositionAction(robot.HOOK_POS_X, robot.HOOK_POS_Y,robot.HOOK_DEGREES,false),0,
//            moveArmEncoderAction(robot.getCurrentArmPosition(),robot.HOOK_ARM_HEIGHT), 1000,
//            moveExtensionEncoderAction(robot.getCurrentExtensionPosition(),robot.HOOK_EXTENSION_POSITION),0,
//            null,0,null);
//        robot.resetDrivePosition();
//
//        //step 2: release hook
//        performActionsWithDelays("RELEASE HOOK",
//                moveArmEncoderAction(robot.getCurrentArmPosition(),robot.HOOK_ARM_HEIGHT_2), 300,
//                moveIntakeTimedAction (false, true,100, false,this),400,
//                moveExtensionEncoderAction(robot.getCurrentExtensionPosition(),0),0,
//                null,0,null);
//        robot.resetDrivePosition();

        //step 3:move to pickup block from floor
        performActionsWithDelays("PICKUP BLOCK GROUND - STEP 1",
                driveToPositionAction(robot.PICKUP_BLOCK_POS_X,robot.PICKUP_BLOCK_POS_Y,0,false),0,
                moveExtensionEncoderAction(robot.getCurrentExtensionPosition(), 0), 500,
                moveArmEncoderAction(robot.getCurrentArmPosition(),robot.DRIVE_ARM_POSITION), 500,
                null,0,null);
        robot.resetDrivePosition();

        //step 4:move extension to pickup block from floor
        performActionsWithDelays("PICKUP BLOCK GROUND - STEP 2",
                moveArmEncoderAction(robot.getCurrentArmPosition(),robot.PICKUP_SAMPLE_ARM_HEIGHT), 0,
                moveExtensionEncoderAction(robot.getCurrentExtensionPosition(), robot.PICKUP_SAMPLE_EXTENSION_POSITION),0,
                null,0,null,0,null);
        robot.resetSampleCaptured();
        //step 5:pickup block from the floor
        performActionsWithDelays("PICKUP BLOCK GROUND - STEP 3",
                driveToPositionAction(robot.PICKUP_BLOCK_POS_INTAKE_X,robot.PICKUP_BLOCK_POS_Y,0,true),0,
                moveIntakeTimedAction (true, false,2500, true,this),100,
                null,0,null,0,null);
        robot.resetDrivePosition();

        if (robot.isSampleCaptured()) {
            //step 6: prepare to drop off the sample
            performActionsWithDelays("DROP OFF SAMPLE - STEP 1",
                    driveToPositionAction(robot.RELEASE_SAMPLE_POS_X, robot.RELEASE_SAMPLE_POS_Y, robot.RELEASE_SAMPLE_DEGREES, false), 0,
                    moveExtensionEncoderAction(robot.getCurrentExtensionPosition(), 0), 500,
                    moveArmEncoderAction(robot.getCurrentArmPosition(), robot.DRIVE_ARM_POSITION), 500,
                    null, 0, null);

            //step 6: drop off the sample arm position
            performActionsWithDelays("DROP OFF SAMPLE - STEP 2",
                    moveArmEncoderAction(robot.getCurrentArmPosition(), robot.RELEASE_SAMPLE_ARM_HEIGHT), 0,
                    moveExtensionEncoderAction(robot.getCurrentExtensionPosition(), robot.RELEASE_SAMPLE_EXTENSION_POSITION), 500,
                    null, 0, null, 0, null);

            //step 6: drop off the sample by lowering arm
            performActionsWithDelays("DROP OFF SAMPLE - STEP 2",
                    moveArmEncoderAction(robot.getCurrentArmPosition(), robot.RELEASE_SAMPLE_ARM_HEIGHT_2), 0,
                    null, 0,
                    null, 0, null, 0, null);

            //step 6: drop off the sample
            performActionsWithDelays("DROP OFF SAMPLE - STEP 3",
                    moveIntakeTimedAction(false, true, 800, false, this), 900,
//                    moveExtensionEncoderAction(robot.getCurrentExtensionPosition(), 0), 0,
                    null, 0, null, 0, null ,0, null);

            //step 7:pickup second block from the floor
            robot.resetDrivePosition();

            performActionsWithDelays("PICKUP BLOCK 2 GROUND - STEP 1",
                    driveToPositionAction(robot.PICKUP_BLOCK_POS_INTAKE_X, robot.PICKUP_BLOCK_2_POS_Y, 0, true), 0,
                    moveExtensionEncoderAction(robot.getCurrentExtensionPosition(), 0), 500,
                    moveArmEncoderAction(robot.getCurrentArmPosition(), robot.DRIVE_ARM_POSITION), 500,
                    null, 0, null);
            robot.resetDrivePosition();

            performActionsWithDelays("PICKUP BLOCK 2 GROUND - STEP 2",
                    moveArmEncoderAction(robot.getCurrentArmPosition(), robot.PICKUP_SAMPLE_ARM_HEIGHT), 0,
                    moveExtensionEncoderAction(robot.getCurrentExtensionPosition(), robot.PICKUP_SAMPLE_EXTENSION_POSITION), 0,
                    null, 0, null, 0, null);

            robot.resetSampleCaptured();
            performActionsWithDelays("PICKUP BLOCK 2 GROUND - STEP 3",
                    driveToPositionAction(robot.PICKUP_BLOCK_POS_INTAKE_X, robot.PICKUP_BLOCK_2_POS_Y, 0, true), 0,
                    moveIntakeTimedAction(true, false, 2500, true, this), 0,
                    null, 0, null, 0, null);

            robot.resetDrivePosition();
            if (robot.isSampleCaptured()) {
                //step 6: prepare to drop off the sample
                performActionsWithDelays("DROP OFF SAMPLE - STEP 1",
                        driveToPositionAction(robot.RELEASE_SAMPLE_POS_X, robot.RELEASE_SAMPLE_POS_Y, robot.RELEASE_SAMPLE_DEGREES, false), 0,
                        moveExtensionEncoderAction(robot.getCurrentExtensionPosition(), 0), 500,
                        moveArmEncoderAction(robot.getCurrentArmPosition(), robot.DRIVE_ARM_POSITION), 500,
                        null, 0, null);

                //step 6: drop off the sample arm position
                performActionsWithDelays("DROP OFF SAMPLE - STEP 2",
                        moveArmEncoderAction(robot.getCurrentArmPosition(), robot.RELEASE_SAMPLE_ARM_HEIGHT), 0,
                        moveExtensionEncoderAction(robot.getCurrentExtensionPosition(), robot.RELEASE_SAMPLE_EXTENSION_POSITION), 500,
                        null, 0, null, 0, null);

                //step 6: drop off the sample by lowering arm
                performActionsWithDelays("DROP OFF SAMPLE - STEP 2",
                        moveArmEncoderAction(robot.getCurrentArmPosition(), robot.RELEASE_SAMPLE_ARM_HEIGHT_2), 0,
                        null, 0,
                        null, 0, null, 0, null);

                //step 6: drop off the sample
                performActionsWithDelays("DROP OFF SAMPLE - STEP 3",
                        moveIntakeTimedAction(false, true, 800, false, this), 900,
//                        moveExtensionEncoderAction(robot.getCurrentExtensionPosition(), 0), 0,
                        null, 0, null, 0, null ,0,null);

            }
            else {
                //did not pickup sample 2
                performActionsWithDelays("RESET FAILED SAMPLE PICKUP",
                        moveExtensionEncoderAction(robot.getCurrentExtensionPosition(), 0),0,
                        null,0,null,0,null,0,null);
            }
        }
        else {
            //did not pickup the sample
            performActionsWithDelays("RESET FAILED SAMPLE PICKUP",
                moveExtensionEncoderAction(robot.getCurrentExtensionPosition(), 0),0,
                null,0,null,0,null,0,null);
        }
        robot.resetDrivePosition();
        //robot.saveRobotPosition(robot.hardwareMap.appContext);//just in case we don't have time to park
        robot.saveRobotPosition();//just in case we don't have time to park
        //step 7: move to park position
        performActionsWithDelays("PARK - STEP 1",
                driveToPositionAction(robot.PARK_LEFT_AUTO_POS_1_X,robot.PARK_LEFT_AUTO_POS_1_Y,0,false),0,
                moveExtensionEncoderAction(robot.getCurrentExtensionPosition(), 0), 500,
                moveArmEncoderAction(robot.getCurrentArmPosition(), robot.DRIVE_ARM_POSITION), 500,
            null,0,null);
        robot.resetDrivePosition();
        robot.saveRobotPosition();//just in case we don't have time to park
        performActionsWithDelays("PARK - STEP 2",
            driveToPositionAction(robot.PARK_LEFT_AUTO_POS_2_X,robot.PARK_LEFT_AUTO_POS_2_Y,robot.PARK_LEFT_AUTO_POS_2_HEADING,false),0,
            moveArmEncoderAction(robot.getCurrentArmPosition(), robot.PARK_ARM_POSITION), 500,
            moveExtensionEncoderAction(robot.getCurrentExtensionPosition(), robot.PARK_EXTENSION_POSITION),0,
            null,0,null);
        robot.resetDrivePosition();
        robot.saveRobotPosition();//just in case we don't have time to park
        performActionsWithDelays("PARK - STEP 3",
            moveArmEncoderAction(robot.getCurrentArmPosition(), robot.PARK_ARM_POSITION_2), 500,
            null,0,null,0,null,0,null);

        //robot.saveRobotPosition(robot.hardwareMap.appContext);
        robot.saveRobotPosition();//just in case we don't have time to park
        closeRobot();
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
}
