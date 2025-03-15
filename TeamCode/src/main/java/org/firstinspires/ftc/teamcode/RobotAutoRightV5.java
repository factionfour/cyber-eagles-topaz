package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Topaz: Auto Drive Position Right V5", group="Robot")
public class RobotAutoRightV5 extends AutoBase4 {

    @Override
    public void runOpMode() {
        initializeHardware(true);
        while (!isStopRequested() && robot.positionTracker.resetPositionCheck(0,120,0)) {
            telemetry.addData("POSITION - Current X",  "%.2f", robot.positionTracker.getXPositionCM());
            telemetry.addData("POSITION - Current Y",  "%.2f", robot.positionTracker.getYPositionCM());
            telemetry.addData("POSITION - Current heading", robot.positionTracker.getHeadingDegrees());
            telemetry.addData(">", "CHARLIE v5 - *NOT READY*.");
            telemetry.update();
            sleep(100);
        }
        telemetry.addData(">", "CHARLIE v5 *READY* - RIGHT AUTO.  Press START.");    //
        telemetry.update();
        waitForStart();

        // Step 1:  position to hook specimen
       // robot.driveToPosition();
        performActionsWithDelays("DRIVE TO POSITION",
            driveToPositionAction(robot.HOOK_POS_X, robot.HOOK_POS_Y,robot.HOOK_DEGREES,false),0,
            moveArmEncoderAction(robot.getCurrentArmPosition(),robot.HOOK_ARM_HEIGHT), 300,
            moveExtensionEncoderAction(robot.getCurrentExtensionPosition(),robot.HOOK_EXTENSION_POSITION),0,
            null,0,null);
            robot.resetDrivePosition();
//        //step 2: release hook
        performActionsWithDelays("RELEASE HOOK",
                moveArmEncoderAction(robot.getCurrentArmPosition(),robot.HOOK_ARM_HEIGHT_2), 0,
                moveIntakeTimedAction (false, true,100, false,this),600,
                moveExtensionEncoderAction(robot.getCurrentExtensionPosition(),0),0,
                null,0,null);

//        //step 3:start the push of the blocks
        performActionsWithDelays("MOVE TO START PUSH",
                moveArmEncoderAction(robot.getCurrentArmPosition(),robot.DRIVE_ARM_POSITION),100,
                driveToPositionAction(robot.PUSH_FIRST_BLOCK_POS_X_0,robot.PUSH_FIRST_BLOCK_POS_Y_0, 0, false),0,
                null,0,null,0,null);
        robot.resetDrivePosition();
//        //step 3:start the push of the blocks
        performActionsWithDelays("PUSH 1 - STEP 1",
                driveToPositionAction(robot.PUSH_FIRST_BLOCK_POS_X_1,robot.PUSH_FIRST_BLOCK_POS_Y_1,0,false),0,
                null,0,null,0,null,0,null);
        robot.resetDrivePosition();
        performActionsWithDelays("PUSH 1 - STEP 2",
                driveToPositionAction(robot.PUSH_FIRST_BLOCK_POS_X_2,robot.PUSH_FIRST_BLOCK_POS_Y_2,0, false),0,
                null,0,null,0,null,0,null);
        robot.resetDrivePosition();
        performActionsWithDelays("PUSH 1 - STEP 3",
                driveToPositionAction(robot.PUSH_FIRST_BLOCK_POS_X_3,robot.PUSH_FIRST_BLOCK_POS_Y_3,0, false),0,
                null,0,null,0,null,0,null);
        robot.resetDrivePosition();
        performActionsWithDelays("PUSH 1 - STEP 4",
                driveToPositionAction(robot.PUSH_FIRST_BLOCK_POS_X_4,robot.PUSH_FIRST_BLOCK_POS_Y_4,0,false),0,
                null,0,null,0,null,0,null);
        robot.resetDrivePosition();
        performActionsWithDelays("PUSH 2 - STEP 1",
                driveToPositionAction(robot.PUSH_SECOND_BLOCK_POS_X_1,robot.PUSH_SECOND_BLOCK_POS_Y_1,0, false),0,
                null,0,null,0,null,0,null);
        robot.resetDrivePosition();
        performActionsWithDelays("PUSH 2 - STEP 2",
                driveToPositionAction(robot.PUSH_SECOND_BLOCK_POS_X_2,robot.PUSH_SECOND_BLOCK_POS_Y_2,0, false),0,
                null,0,null,0,null,0,null);
//        robot.resetDrivePosition();
//        performActionsWithDelays("PUSH 2 - STEP 3",
//                driveToPositionAction(robot.PUSH_SECOND_BLOCK_POS_X_3,robot.PUSH_SECOND_BLOCK_POS_Y_3,0,false),0,
//                null,0,null,0,null,0,null);
//        robot.resetDrivePosition();
//        performActionsWithDelays("PUSH 2 - STEP 4",
//                driveToPositionAction(robot.PUSH_SECOND_BLOCK_POS_X_4,robot.PUSH_SECOND_BLOCK_POS_Y_2,0,false),0,
//                null,0,null,0,null,0,null);
        robot.resetDrivePosition();

        performActionsWithDelays("PICKUP 2 - STEP 1",
            driveToPositionAction(robot.PICKUP_SAMPLE_POS_X,robot.PICKUP_SAMPLE_POS_Y,robot.PICKUP_SAMPLE_DEGREES,false),0,
            null,0,null,0,null,0,null);
        sleep(2000);
        robot.resetDrivePosition();

        robot.resetSampleCaptured();
        performActionsWithDelays("PICKUP 2 - STEP 2",
            moveArmEncoderAction(robot.getCurrentArmPosition(),robot.PICKUP_SAMPLE_ARM_HEIGHT),0,
            moveExtensionEncoderAction(robot.getCurrentExtensionPosition(),robot.PICKUP_SAMPLE_EXTENSION_POSITION),0,
                null,0,null,0,null);

        performActionsWithDelays("PICKUP 2 - STEP 3",
                driveToPositionAction(robot.PICKUP_SAMPLE_POS_INTAKE_X,robot.PICKUP_SAMPLE_POS_Y,robot.PICKUP_SAMPLE_DEGREES,true),0,
                moveIntakeTimedAction (true, false,2000, true,this),0,
                null,0,null,0,null);
        robot.resetDrivePosition();

        if (robot.isSampleCaptured()) {
            //pickup success
            robot.resetSampleCaptured();
            performActionsWithDelays("HOOK 2 - STEP 1",
                    moveExtensionEncoderAction(robot.getCurrentExtensionPosition(),robot.EXTENSION_MIN_POSITION),50,
                    moveArmEncoderAction(robot.getCurrentArmPosition(),robot.DRIVE_ARM_POSITION),0,
                    null,0,null,0,null);

            performActionsWithDelays("HOOK 2 - STEP 2",
                    driveToPositionAction(robot.HOOK_POS_X, robot.HOOK_POS_Y - 10,robot.HOOK_DEGREES,false),0,
                    moveArmEncoderAction(robot.getCurrentArmPosition(),robot.HOOK_ARM_HEIGHT), 2000,
                    moveExtensionEncoderAction(robot.getCurrentExtensionPosition(),robot.HOOK_EXTENSION_POSITION),3000,
                    null,0,null);
            robot.resetDrivePosition();

            performActionsWithDelays("HOOK 2 - RELEASE HOOK",
                    moveArmEncoderAction(robot.getCurrentArmPosition(),robot.HOOK_ARM_HEIGHT_2), 0,
                    moveIntakeTimedAction (false, true,100, false,this),600,
                    moveExtensionEncoderAction(robot.getCurrentExtensionPosition(),0),0,
                    null,0,null);

        }
        //robot.saveRobotPosition(robot.hardwareMap.appContext);//just in case we don't have time to park.
        //robot.saveRobotPosition();//just in case we don't have time to park
//        else
//        {
//            //backup and try again
//            performActionsWithDelays("PICKUP 2 - STEP 1",
//                    driveToPositionAction(robot.PICKUP_SAMPLE_POS_NOPICKUP_X,robot.PICKUP_SAMPLE_POS_Y,robot.PICKUP_SAMPLE_DEGREES,false),0,
//                    null,0,null,0,null,0,null);
//            robot.resetDrivePosition();
//            sleep(2000);
//
//            robot.resetSampleCaptured();
//            performActionsWithDelays("PICKUP 2 - STEP 2",
//                    moveArmEncoderAction(robot.getCurrentArmPosition(),robot.PICKUP_SAMPLE_ARM_HEIGHT),0,
//                    moveExtensionEncoderAction(robot.getCurrentExtensionPosition(),robot.PICKUP_SAMPLE_EXTENSION_POSITION),0,
//                    null,0,null,0,null);
//
//            performActionsWithDelays("PICKUP 2 - STEP 3",
//                    driveToPositionAction(robot.PICKUP_SAMPLE_POS_INTAKE_X,robot.PICKUP_SAMPLE_POS_Y,robot.PICKUP_SAMPLE_DEGREES,true),0,
//                    moveIntakeTimedAction (true, false,1500, true,this),0,
//                    null,0,null,0,null);
//            robot.resetDrivePosition();
//
//            if (robot.isSampleCaptured()) {
//                //pickup success
//                robot.resetSampleCaptured();
//                performActionsWithDelays("HOOK 2 - STEP 1",
//                        moveExtensionEncoderAction(robot.getCurrentExtensionPosition(), robot.EXTENSION_MIN_POSITION), 200,
//                        moveArmEncoderAction(robot.getCurrentArmPosition(), robot.DRIVE_ARM_POSITION), 0,
//                        null, 0, null, 0, null);
//
//                performActionsWithDelays("HOOK 2 - STEP 2",
//                        driveToPositionAction(robot.HOOK_POS_X, robot.HOOK_POS_Y, robot.HOOK_DEGREES, false), 0,
//                        moveArmEncoderAction(robot.getCurrentArmPosition(), robot.HOOK_ARM_HEIGHT), 300,
//                        moveExtensionEncoderAction(robot.getCurrentExtensionPosition(), robot.HOOK_EXTENSION_POSITION), 0,
//                        null, 0, null);
//                robot.resetDrivePosition();
//
//                performActionsWithDelays("HOOK 2 - RELEASE HOOK",
//                        moveArmEncoderAction(robot.getCurrentArmPosition(), robot.HOOK_ARM_HEIGHT_2), 300,
//                        moveIntakeTimedAction(false, true, 100, false, this), 0,
//                        moveExtensionEncoderAction(robot.getCurrentExtensionPosition(), 0), 500,
//                        null, 0, null);
//
//            }
//
//        }
        //park the robot
        performActionsWithDelays("PARK - STEP 1",
                moveArmEncoderAction(robot.getCurrentArmPosition(), robot.DRIVE_ARM_POSITION), 0,
                moveExtensionEncoderAction(robot.getCurrentExtensionPosition(), 0), 0,
                null, 0, null,0,null);

        robot.resetDrivePosition();
        performActionsWithDelays("PARK - STEP 2",
                driveToPositionAction(robot.PUSH_SECOND_BLOCK_POS_X_3,robot.PUSH_SECOND_BLOCK_POS_Y_3,0,false),0,
                null,0,null,0,null,0,null);
        robot.resetDrivePosition();
        performActionsWithDelays("PARK - STEP 3",
                driveToPositionAction(robot.PUSH_SECOND_BLOCK_POS_X_4,robot.PUSH_SECOND_BLOCK_POS_Y_2,0,false),0,
                null,0,null,0,null,0,null);

        //robot.saveRobotPosition(robot.hardwareMap.appContext);
        robot.saveRobotPosition();//just in case we don't have time to park
        closeRobot();
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
}
