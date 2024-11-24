
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Robot: Auto Drive By Time Position 1", group="Robot")
public class RobotAutoDriveByTimePos1_Linear extends AutoBase {
    @Override
    public void runOpMode() {
        initializeHardware();
        // Step 1:  Drive forward for 3 seconds
        //driveForward(3000,100);

//      Step 2:  Straife right for 1.3 seconds
        //strafeLeft(1300,100);

        moveArm(100,1000,100);

        driveForward(400,200);

        moveExtension(500,1000,100);

        moveArm(200,1000,100);

        moveExtension(1500,1000,100);
        moveArm(400,1000,100);
        intakeClaw(1000,100);
        closeRobot();
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
