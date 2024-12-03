
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Topaz: Auto Drive Position Right", group="Robot")
public class RobotAutoRight extends AutoBase {
    @Override
    public void runOpMode() {
        initializeHardware();

        setInitialPosition();
        // Step 1:  Drive strafeleft 360mm
        strafeLeft(837,200);
        driveForward(930,200);
        moveArm(HOOK_ARM_HEIGHT,1500,400);
        moveExtension(HOOK_EXTENSION_POSITION,1500,200);
        moveArm(HOOK_RELEASE_ARM_HEIGHT,1500,200);
        moveExtension(0,2000,200);
        moveArm(0,2000,0);
        strafeRight(1575,200);

        //first block
        driveForward(1400,200);
        strafeRight(450,200);
        driveBackward(1600,200);

        //second block
        driveForward(1600,200);
        strafeRight(450,200);
        driveBackward(1600,200);

        //third block
        driveForward(1600,200);
        //strafeRight(310,200);
        //driveBackward(1600,200);


        driveForward(500,200);
        //strafeLeft(1000,200);

        turnRight(180,200);

        closeRobot();
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}