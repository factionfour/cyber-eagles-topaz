
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name="Z: Test Strafe Left 1 second", group="Robot")
public class Test_StraifeLeft extends AutoBase {
    @Override
    public void runOpMode() {
        initializeHardware();
        setInitialPosition();
        // Step 1:  Drive forward for 1 second
        strafeLeft(1000,200);
        closeRobot();
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
