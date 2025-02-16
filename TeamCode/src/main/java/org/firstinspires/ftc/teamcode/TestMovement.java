package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Topaz: Test Movement", group="Robot")
public class TestMovement extends AutoBase4 {

    @Override
    public void runOpMode() {
        initializeHardware(true);
        while (!isStopRequested() && robot.positionTracker.resetPositionCheck(0,240,0)) {
            telemetry.addData("POSITION - Current X",  "%.2f", robot.positionTracker.getXPositionCM());
            telemetry.addData("POSITION - Current Y",  "%.2f", robot.positionTracker.getYPositionCM());
            telemetry.addData("POSITION - Current heading", robot.positionTracker.getHeadingDegrees());
            telemetry.addData(">", "CHARLIE 5 - *NOT READY*.");
            telemetry.update();
            sleep(100);
        }
        telemetry.addData(">", "CHARLIE 5 *READY* - TEST MOVEMENT.  Press START.");    //
        telemetry.update();
        waitForStart();

        robot.resetDrivePosition();

        //step 3:move to pickup block from floor
        performActionsWithDelays("PICKUP BLOCK GROUND - STEP 1",
                moveArmEncoderAction(robot.getCurrentArmPosition(),robot.DRIVE_ARM_POSITION), 0,
                driveToPositionAction(15,0,0,false),0,
                null,0,null,0,null);
        robot.resetDrivePosition();

        //step 4:move extension to pickup block from floor
        performActionsWithDelays("PICKUP BLOCK GROUND - STEP 1",
                moveExtensionEncoderAction(robot.getCurrentExtensionPosition(), robot.PICKUP_SAMPLE_EXTENSION_POSITION),0,
                null,0,null,0,null,0,null);

        //step 5:pickup block from the floor
        performActionsWithDelays("PICKUP BLOCK GROUND - STEP 2",
                driveToPositionAction(35,0,0,true),0,
                moveIntakeTimedAction (true, false,2500, true,this),0,
                null,0,null,0,null);
        robot.resetDrivePosition();

        if (robot.isSampleCaptured()) {
            //step 6: prepare to drop off the sample
            performActionsWithDelays("DROP OFF SAMPLE - STEP 1",
                    moveExtensionEncoderAction(robot.getCurrentExtensionPosition(), 0),300,
                    moveArmEncoderAction(robot.getCurrentArmPosition(), robot.DRIVE_ARM_POSITION), 0,
                    null,0,
                    null,0,null);


        }
        else {
            //did not pickup the sample
            performActionsWithDelays("RESET FAILED SAMPLE PICKUP",
                    moveExtensionEncoderAction(robot.getCurrentExtensionPosition(), 0),0,
                    null,0,null,0,null,0,null);
        }

        closeRobot();
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
}
