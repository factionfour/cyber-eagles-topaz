
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Robot: Test Forward 1500mm", group="Robot")
public class Test_DriveFront2 extends AutoBase {
    @Override
    public void runOpMode() {
        initializeHardware();
        // Step 1:  Drive forward for 1 second
        driveForwardMM(1500,200);
        closeRobot();
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
