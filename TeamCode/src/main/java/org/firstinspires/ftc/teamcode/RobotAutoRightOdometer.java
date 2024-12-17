
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Topaz: Auto Drive Position Right V3", group="Robot")
public class RobotAutoRightOdometer extends AutoBase2 {
    @Override
    public void runOpMode() {
        initializeHardware();

        positionTracker.resetPosition(400,0);

        telemetry.addData("Path", "Start");
        telemetry.update();

        // Step 1:  position to hook specimen
        strafeLeft(737,200);
        performActionsWithDelays(
            () -> driveForward(840,0),0,
            () -> moveArmEncoder(HOOK_ARM_HEIGHT,0), 300,
            () -> moveExtensionEncoder(HOOK_EXTENSION_POSITION,0),0,
            () -> {},0,
            () -> {});

        //step 2: release hook
        performActionsWithDelays(
                () -> outputClaw(225,0),150,
                () -> moveArmEncoder(HOOK_RELEASE_ARM_HEIGHT,0), 300,
                () -> moveExtensionEncoder(HOOK_RELEASE_EXTENSION_POSITION,200), 0,
                () -> {},0,
                () -> {});

        //step 3: move to the push position
        performActionsWithDelays(
                () -> moveArmEncoder(TRAVEL_ARM_HEIGHT,0),0,
                () -> moveExtensionEncoder(TRAVEL_EXTENSION_HEIGHT,0), 150,
                () -> strafeRight(1525,200),0,
                () -> {}, 0,
                () -> {});

        //step 4: push first block
        rotateToHeading(initRobotHeading,100);//straighten out
        driveForward(1350,100);
        strafeRight(400,100);
        driveBackward(1500,100);
        rotateToHeading(initRobotHeading,100);//straighten out

        //step 5: push second block
        driveForward(1500,1000);
        strafeRight(400,200);
        driveBackward(1500,1500);
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
