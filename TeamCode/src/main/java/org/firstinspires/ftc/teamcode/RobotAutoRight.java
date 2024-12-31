
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Topaz: Auto Drive Position Right V1", group="Robot")
public class RobotAutoRight extends AutoBase {
    @Override
    public void runOpMode() {
        initializeHardware();

        setInitialPosition();

        telemetry.addData("Path", "Start");
        telemetry.update();

        // Step 1:  Drive strafeleft 360mm
        strafeLeft(737,200);
        driveForward(840,200);
        moveArm(HOOK_ARM_HEIGHT,3500,400);
        moveExtension(HOOK_EXTENSION_POSITION,2500,200);
        outputClaw(225,100);
        moveArm(HOOK_RELEASE_ARM_HEIGHT,2000,600);

        moveExtension(0,2500,200);
        moveArm(0,3500,0);
        rotateToHeading(initRobotHeading,200);//straighten out
        strafeRight(1525,200);

        //first block
        driveForward(1350,200);
        strafeRight(400,200);
        driveBackward(1550,200);
        rotateToHeading(initRobotHeading,200);//straighten out

        //second block
        driveForward(1550,1000);
        strafeRight(400,200);
        driveBackward(1550,1500);
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
        moveExtension(0,2500,200);
        //driveForward(200,200);//park outside the sample cage

        closeRobot();
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
