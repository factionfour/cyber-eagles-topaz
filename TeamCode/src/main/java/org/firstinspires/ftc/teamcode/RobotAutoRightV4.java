package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Topaz: Auto Drive Position Right V4", group="Robot")
public class RobotAutoRightV4 extends AutoBase4 {

    @Override
    public void runOpMode() {
        initializeHardware(0,118);

        // Step 1:  position to hook specimen
       // robot.driveToPosition();
        performActionsWithDelays("DRIVE TO POSITION",

            () -> robot.driveToPosition(robot.HOOK_POS_X, robot.HOOK_POS_Y,robot.HOOK_DEGREES),0,
            () -> robot.moveArmEncoder(robot.getCurrentArmPosition(),robot.HOOK_ARM_HEIGHT), 300,
            () -> robot.moveExtensionEncoder(robot.getCurrentExtensionPosition(),robot.HOOK_EXTENSION_POSITION),0,
            () -> {},0,
            () -> {});

//        //step 2: release hook
//        performActionsWithDelays("RELEASE HOOK",
//                () -> robot.moveIntake(false,true),150,
//                () -> robot.moveExtensionEncoder(robot.getCurrentExtensionPosition(),0),0,
//                () -> {},0,
//                () -> {},0,
//                () -> {});
//
//        //step 3: move to the push position
//        performActionsWithDelays("MOVE TO PUSH 1",
//                () -> robot.moveArmEncoder(robot.getCurrentArmPosition(),0),0,
//                () -> robot.moveExtensionEncoder(robot.getCurrentExtensionPosition(),0), 150,
//                () -> robot.driveToPosition(robot.PUSH_FIRST_BLOCK_POS_X_1,-robot.PUSH_FIRST_BLOCK_POS_Y_1,0),0,
//                () -> {}, 0,
//                () -> {});
//
//        robot.driveToPosition(robot.PUSH_FIRST_BLOCK_POS_X_2,robot.PUSH_FIRST_BLOCK_POS_Y_2,0);
//        robot.driveToPosition(robot.PUSH_FIRST_BLOCK_POS_X_3,robot.PUSH_FIRST_BLOCK_POS_Y_3,0);
//        robot.driveToPosition(robot.PUSH_FIRST_BLOCK_POS_X_4,robot.PUSH_FIRST_BLOCK_POS_Y_4,0);
//
//        robot.driveToPosition(robot.PUSH_SECOND_BLOCK_POS_X_1,robot.PUSH_SECOND_BLOCK_POS_Y_1,0);
//        robot.driveToPosition(robot.PUSH_SECOND_BLOCK_POS_X_2,robot.PUSH_SECOND_BLOCK_POS_Y_2,0);
//        robot.driveToPosition(robot.PUSH_SECOND_BLOCK_POS_X_3,robot.PUSH_SECOND_BLOCK_POS_Y_3,0);

        //park the robot
        robot.positionTracker.saveRobotPosition(robot.hardwareMap.appContext);
        closeRobot();
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
}
