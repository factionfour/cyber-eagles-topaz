package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Topaz: Auto Drive Position Left V4", group="Robot")
public class RobotAutoLeftV4 extends AutoBase4 {

    @Override
    public void runOpMode() {
        initializeHardware(0,118);

        // Step 1:  position to hook specimen
        performActionsWithDelays("DRIVE TO POSITION",
            driveToPositionAction(robot.HOOK_POS_X, robot.HOOK_POS_Y,robot.HOOK_DEGREES),0,
            moveArmEncoderAction(robot.getCurrentArmPosition(),robot.HOOK_ARM_HEIGHT), 300,
            moveExtensionEncoderAction(robot.getCurrentExtensionPosition(),robot.HOOK_EXTENSION_POSITION),0,
            null,0,null);
        robot.resetDrivePosition();

//        //step 2: release hook
        performActionsWithDelays("RELEASE HOOK",
                moveIntakeTimedAction (false, true,150, this),0,
                moveExtensionEncoderAction(robot.getCurrentExtensionPosition(),0),700,
                driveToPositionAction(robot.POST_HOOK_POS_X,robot.POST_HOOK_POS_Y,0),0,
                null,0, null);

//        //step 3:pickup block from floor
        performActionsWithDelays("PICKUP BLOCK GROUND - STEP 1",
                moveArmEncoderAction(robot.getCurrentArmPosition(),robot.PICKUP_SAMPLE_ARM_HEIGHT), 0,
                driveToPositionAction(robot.PICKUP_BLOCK_POS_X,robot.PICKUP_BLOCK_POS_Y,0),750,
                moveExtensionEncoderAction(robot.getCurrentExtensionPosition(), robot.PICKUP_SAMPLE_EXTENSION_POSITION), 0,
                null,0,null);
        robot.resetDrivePosition();

        //step 4: move the robot forward and start the intake

        //step 5: move the extension back in (partially only), and move the arm up to driving position

        //step 6: drop the item in the bucket

        //step 7: move to park position

        robot.positionTracker.saveRobotPosition(robot.hardwareMap.appContext);
        closeRobot();
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
}
