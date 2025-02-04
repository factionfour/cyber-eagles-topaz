package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Topaz: Auto Drive Position Left V4", group="Robot")
public class RobotAutoLeftV4 extends AutoBase4 {

    @Override
    public void runOpMode() {
        initializeHardware();
        robot.positionTracker.resetPosition(0,240,0);
        waitForStart();
        robot.positionTracker.resetPosition(0,240,0);
        sleep(1000);

        // Step 1:  position to hook specimen
        performActionsWithDelays("DRIVE TO POSITION",
            driveToPositionAction(robot.HOOK_POS_X, robot.HOOK_POS_Y,robot.HOOK_DEGREES),0,
            moveArmEncoderAction(robot.getCurrentArmPosition(),robot.HOOK_ARM_HEIGHT), 300,
            moveExtensionEncoderAction(robot.getCurrentExtensionPosition(),robot.HOOK_EXTENSION_POSITION),0,
            null,0,null);
        robot.resetDrivePosition();

        //step 2: release hook
        performActionsWithDelays("RELEASE HOOK",
                moveArmEncoderAction(robot.getCurrentArmPosition(),robot.HOOK_ARM_HEIGHT_2), 300,
                moveIntakeTimedAction (false, true,300, this),0,
                moveExtensionEncoderAction(robot.getCurrentExtensionPosition(),0),500,
                null,0,null);

        robot.resetDrivePosition();

        //step 3:move to pickup block from floor
        performActionsWithDelays("PICKUP BLOCK GROUND - STEP 1",
                moveArmEncoderAction(robot.getCurrentArmPosition(),robot.DRIVE_ARM_POSITION), 0,
                driveToPositionAction(robot.PICKUP_BLOCK_POS_X,robot.PICKUP_BLOCK_POS_Y,0),0,
                null,0,null,0,null);

        robot.resetDrivePosition();

        //step 4:move extension to pickup block from floor
        performActionsWithDelays("PICKUP BLOCK GROUND - STEP 1",
                moveExtensionEncoderAction(robot.getCurrentExtensionPosition(), robot.PICKUP_SAMPLE_EXTENSION_POSITION),0,
                null,0,null,0,null,0,null);

        //step 5:pickup block from the floor
        performActionsWithDelays("PICKUP BLOCK GROUND - STEP 3",
                driveToPositionAction(robot.PICKUP_BLOCK_POS_INTAKE_X,robot.PICKUP_BLOCK_POS_Y,0),0,
                moveIntakeTimedAction (true, false,350, this),0,
                null,0,null,0,null);
        robot.resetDrivePosition();

        if (robot.isSampleCaptured()) {
            //step 6: prepare to drop off the sample
            performActionsWithDelays("DROP OFF SAMPLE - STEP 1",
                    moveExtensionEncoderAction(robot.getCurrentExtensionPosition(), 0),300,
                    moveArmEncoderAction(robot.getCurrentArmPosition(), robot.DRIVE_ARM_POSITION), 0,
                    driveToPositionAction(robot.RELEASE_SAMPLE_POS_X,robot.RELEASE_SAMPLE_POS_Y,robot.RELEASE_SAMPLE_DEGREES),0,
                    null,0,null);
            robot.resetDrivePosition();

            //step 6: drop off the sample arm position
            performActionsWithDelays("DROP OFF SAMPLE - STEP 2",
                    moveArmEncoderAction(robot.getCurrentArmPosition(), robot.RELEASE_SAMPLE_ARM_HEIGHT), 300,
                    moveExtensionEncoderAction(robot.getCurrentExtensionPosition(), robot.RELEASE_SAMPLE_EXTENSION_POSITION),500,
                    null,0,null,0,null);

            //step 6: drop off the sample
            performActionsWithDelays("DROP OFF SAMPLE - STEP 3",
                    moveIntakeTimedAction(false, true,600,this), 1500,
                    moveExtensionEncoderAction(robot.getCurrentExtensionPosition(), 0),0,
                    null,0,null,0,null);

        }
        else {
            //did not pickup the sample
            performActionsWithDelays("RESET FAILED SAMPLE PICKUP",
                    moveExtensionEncoderAction(robot.getCurrentExtensionPosition(), 0),0,
                    null,0,null,0,null,0,null);
        }

        //step 7: move to park position
        performActionsWithDelays("PARK - STEP 1",
                moveArmEncoderAction(robot.getCurrentArmPosition(), robot.DRIVE_ARM_POSITION), 0,
                driveToPositionAction(robot.PARK_LEFT_AUTO_POS_1_X,robot.PARK_LEFT_AUTO_POS_1_Y,0),0,
                null,0,null,0,null);

        robot.resetDrivePosition();

        performActionsWithDelays("PARK - STEP 2",
                driveToPositionAction(robot.PARK_LEFT_AUTO_POS_2_X,robot.PARK_LEFT_AUTO_POS_2_Y,0),0,
                null,0,null,0,null,0,null);
        robot.resetDrivePosition();

        robot.positionTracker.saveRobotPosition(robot.hardwareMap.appContext);
        closeRobot();
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
}
