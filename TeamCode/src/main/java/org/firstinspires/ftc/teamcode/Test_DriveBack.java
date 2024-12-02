
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Z: Test Backward 1 second", group="Robot")
public class Test_DriveBack extends AutoBase {
    @Override
    public void runOpMode() {
        initializeHardware();
        setInitialPosition();
        // Step 1:  Drive forward for 1 second
        driveBackward(1000,200);
        closeRobot();
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
