
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Robot: Auto Drive By Time Position 1", group="Robot")
public class RobotAutoBlueLeft extends AutoBase {
    @Override
    public void runOpMode() {
        initializeHardware();
        // Step 1:  Drive forward for 30 CM
        driveForwardMM(50,100);
        strafeRightMM(500,100);
        driveForwardMM(200,100);
//        //prepare the hook
//        moveArm(HOOK_ARM_HEIGHT,1000,100);
//        moveExtension(HOOK_EXTENSION_POSITION,1000,500);
//
//        //hook the specimen
//        outputClaw(400,100);
//        moveArm(HOOK_RELEASE_ARM_HEIGHT,1000,100);
//        moveExtension(HOOK_RELEASE_EXTENSION_POSITION,1000,500);
//        driveBackwardMM(50,100);
//
//        //now move to start sample collection
//        moveExtension(0,1000,100);
//        moveArm(100,1000,0);
//        turnLeft(600,0);
//        driveForwardMM(600,100);
//        turnRight(600,100);
//        driveForwardMM(1500,00);
//
//        //start sample collection
//        strafeLeftMM(200,100);
//        driveBackwardMM(2000,100);
//        driveForwardMM(2000,00);
//        strafeLeftMM(200,100);
//        driveBackwardMM(2000,100);
//        driveForwardMM(2000,00);
//        strafeLeftMM(200,100);
//        driveBackwardMM(2000,100);
//
//        //prepare the robot for sample collection
//        driveForwardMM(500,100);
//        turnRight(1200,100);
//        driveForwardMM(500,100);

        closeRobot();
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
