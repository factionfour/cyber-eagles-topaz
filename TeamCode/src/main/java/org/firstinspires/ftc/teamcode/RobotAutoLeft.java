
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Topaz: Auto Drive Position Left", group="Robot")
public class RobotAutoLeft extends AutoBase {
    @Override
    public void runOpMode() {
        initializeHardware();
        setInitialPosition();

        //hook the first specimen
        strafeRight(737,200);
        driveForward(880,200);
        moveArm(HOOK_ARM_HEIGHT,3500,400);
        moveExtension(HOOK_EXTENSION_POSITION,2500,200);
        moveArm(HOOK_RELEASE_ARM_HEIGHT,2000,200);
        moveExtension(0,2500,200);
        moveArm(0,3500,0);
        rotateToHeading(initRobotHeading,200);//straighten out

        //park the robot
        strafeLeft(737,200);
        driveBackward(880,200);

        closeRobot();
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
