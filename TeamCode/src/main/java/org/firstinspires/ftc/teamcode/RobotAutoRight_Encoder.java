
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Topaz: Auto Drive Position Right (Encoder)", group="Robot")
public class RobotAutoRight_Encoder extends AutoBase {
    @Override
    public void runOpMode() {
        initializeHardware();

        setInitialPosition();
        // Step 1:  Drop off first specimen (pre-loaded)
        strafeLeft(737,200);
        driveForward(880,200);
        moveArmEncoder(HOOK_ARM_HEIGHT,3500,400);
        moveExtensionEncoder(HOOK_EXTENSION_POSITION,2500,200);
        moveArmEncoder(HOOK_RELEASE_ARM_HEIGHT,2000,200);
        moveExtensionEncoder(0,2500,200);
        moveArmEncoder(0,3500,0);
        rotateToHeading(initRobotHeading,200);//straighten out
        strafeRight(1525,200);

        //move back first block
        driveForward(1400,200);
        strafeRight(400,200);
        driveBackward(1600,200);
        rotateToHeading(initRobotHeading,200);//straighten out

        //move back second block
        driveForward(1600,200);
        strafeRight(400,200);
        driveBackward(1600,1500);
        //rotateToHeading(initRobotHeading,200);//straighten out

//        //retrieve block as second specimen
//        driveForward(400,200);
//        turnLeft(180,200);
//        moveArmEncoder(PICKUP_SPECIMEN_ARM_HEIGHT,2000,0);
//        moveExtensionEncoder(PICKUP_SPECIMEN_EXTENSION_POSITION,2000,200);
//        intakeClaw(3000,500);
//
//        //go back to hook & set second specimen
//        moveExtension(0,2000,200);
//        driveBackward(200,1500);
//        turnRight(180,200);
//        strafeLeft(1200,200);
//        rotateToHeading(initRobotHeading,200);//straighten out
//
//        moveArmEncoder(HOOK_ARM_HEIGHT,3500,400);
//        moveExtensionEncoder(HOOK_EXTENSION_POSITION,2500,200);
//        moveArmEncoder(HOOK_RELEASE_ARM_HEIGHT,2000,200);

        //park the robot
        moveExtensionEncoder(0,2500,200);
        driveForward(200,200);//park outside the sample cage

        closeRobot();
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
