
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Topaz: Auto Drive Position Left", group="Robot")
public class RobotAutoLeft extends AutoBase {
    @Override
    public void runOpMode() {
        initializeHardware();
        setInitialPosition();

        telemetry.addData("Path", "Start");
        telemetry.update();

        telemetry.addData("Hook", "Start");
        telemetry.update();

        //hook the first specimen
        strafeRight(1400,200);
        driveForward(880,200);
        moveArm(HOOK_ARM_HEIGHT,3500,400);
        moveExtension(HOOK_EXTENSION_POSITION,2500,200);
        outputClaw(225,100);
        moveArm(HOOK_RELEASE_ARM_HEIGHT,2000,200);
        moveExtension(0,2500,200);
        moveArm(0,3500,0);
        rotateToHeading(initRobotHeading,200);//straighten out

        telemetry.addData("Hook Specimen", "Complete");
        telemetry.update();

        //park the robot
        strafeLeft(1500,200);
        rotateToHeading(initRobotHeading,200);//straighten out
        driveForward(1350,200);
        rotateToHeading(initRobotHeading,200);//straighten out
        turnRight(90,500);
        driveForward(100,0);
        moveArm(HOOK_RELEASE_ARM_HEIGHT,2500,200);
        moveExtension(PARK_EXTENSION_POSITION,2500,200);
        moveArm(PARK_ARM_HEIGHT,1000,200);


        //driveBackward(880,200);
        //closeRobot();
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
