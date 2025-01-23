
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Topaz: Auto Drive Position Right V3", group="Robot")
public class RobotAutoRightV3 extends AutoBase4 {

    @Override
    public void runOpMode() {

        // Step 1:  position to hook specimen
       // robot.driveToPosition();
        performActionsWithDelays(
            () -> robot.driveToPosition(robot.HOOK_POS_X, robot.HOOK_POS_Y,robot.HOOK_DEGREES),0,
            () -> robot.moveArmEncoder(robot.getCurrentArmPosition(),robot.HOOK_ARM_HEIGHT), 300,
            () -> robot.moveExtensionEncoder(robot.getCurrentExtensionPosition(),robot.HOOK_EXTENSION_POSITION),0,
            () -> {},0,
            () -> {});

        //step 2: release hook
        performActionsWithDelays(
                () -> robot.moveIntake(false,true),150,
                () -> robot.moveExtensionEncoder(robot.getCurrentExtensionPosition(),0),0,
                () -> {},0,
                () -> {},0,
                () -> {});

        //step 3: move to the push position
        performActionsWithDelays(
                () -> robot.moveArmEncoder(robot.getCurrentArmPosition(),0),0,
                () -> robot.moveExtensionEncoder(robot.getCurrentExtensionPosition(),0), 150,
                () -> robot.driveToPosition(robot.PUSH_FIRST_BLOCK_POS_X_1,-robot.PUSH_SECOND_BLOCK_POS_Y_1,0),0,
                () -> {}, 0,
                () -> {});

        performActionsWithDelays(
                () -> robot.driveToPosition(robot.PUSH_FIRST_BLOCK_POS_X_2,robot.PUSH_FIRST_BLOCK_POS_Y_2,0),0,
                () -> {}, 0,
                () ->{}, 0,
                () -> {}, 0,
                () -> {});


        robot.driveToPosition(robot.PUSH_FIRST_BLOCK_POS_X_2,robot.PUSH_FIRST_BLOCK_POS_Y_2,0);
        robot.driveToPosition(robot.PUSH_FIRST_BLOCK_POS_X_3,robot.PUSH_FIRST_BLOCK_POS_Y_3,0);
        robot.driveToPosition(robot.PUSH_FIRST_BLOCK_POS_X_4,robot.PUSH_FIRST_BLOCK_POS_Y_4,0);

        robot.driveToPosition(robot.PUSH_SECOND_BLOCK_POS_X_1,robot.PUSH_SECOND_BLOCK_POS_Y_1,0);
        robot.driveToPosition(robot.PUSH_SECOND_BLOCK_POS_X_2,robot.PUSH_SECOND_BLOCK_POS_Y_2,0);
        robot.driveToPosition(robot.PUSH_SECOND_BLOCK_POS_X_3,robot.PUSH_SECOND_BLOCK_POS_Y_3,0);

        //step 4: push first block
//        rotateToHeading(positionTracker.initHeading,100);//straighten out
//        driveForward(1350,100);
//        strafeRight(400,100);
//        driveBackward(1500,100);
//        rotateToHeading(positionTracker.initHeading,100);//straighten out
//
//        //step 5: push second block
//        driveForward(1500,1000);
//        strafeRight(400,200);
//        driveBackward(1500,1500);
//        rotateToHeading(initRobotHeading,200);//straighten out
//
//        //retrieve first block
//        driveForward(400,200);
//        turnLeft(180,200);
//        strafeRight(400,200);
//        driveBackward(300,0);
//        moveArm(PICKUP_SPECIMEN_ARM_HEIGHT,2000,0);
//        moveExtension(PICKUP_SPECIMEN_EXTENSION_POSITION,2000,200);
//        intakeClaw(3000,500);
//
//        //go back to hook & set second specimen
//        moveExtension(0,2000,200);
//        //driveBackward(200,1500);
//        turnRight(180,200);
//        strafeLeft(1200,200);
//        rotateToHeading(initRobotHeading,200);//straighten out
//
//        moveArm(HOOK_ARM_HEIGHT,3500,400);
//        moveExtension(HOOK_EXTENSION_POSITION,2500,200);
//        moveArm(HOOK_RELEASE_ARM_HEIGHT,2000,200);

        //park the robot
        //driveForward(200,200);//park outside the sample cage

        closeRobot();
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
