package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Topaz: Test straight strafe", group="Robot")
public class RotationTest extends AutoBase4 {

    @Override
    public void runOpMode() {
        initializeHardware(true);
        while (!isStopRequested() && robot.positionTracker.resetPositionCheck(0,0,0)) {
            telemetry.addData("POSITION - Current X",  "%.2f", robot.positionTracker.getXPositionCM());
            telemetry.addData("POSITION - Current Y",  "%.2f", robot.positionTracker.getYPositionCM());
            telemetry.addData("POSITION - Current heading", robot.positionTracker.getHeadingDegrees());
            telemetry.addData(">", "CHARLIE v5 - *NOT READY*.");
            telemetry.update();
            sleep(100);
        }
        telemetry.addData(">", "CHARLIE v5 *READY* - Rotate test.  Press START.");    //
        telemetry.update();
        waitForStart();

        // Step 1:  position to hook specimen
       // robot.driveToPosition();
        performActionsWithDelays("DRIVE TO POSITION",
            driveToPositionAction(10, 30,0,false),0,
            moveArmEncoderAction(robot.getCurrentArmPosition(),0), 300,
            moveExtensionEncoderAction(robot.getCurrentExtensionPosition(),0),0,
            null,0,null);
            robot.resetDrivePosition();

        //robot.saveRobotPosition(robot.hardwareMap.appContext);
        robot.saveRobotPosition();//just in case we don't have time to park
        closeRobot();
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
}
