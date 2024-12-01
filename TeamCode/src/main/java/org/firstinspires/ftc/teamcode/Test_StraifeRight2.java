
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Z: Test Strafe Right 1500mm", group="Robot")
public class Test_StraifeRight2 extends AutoBase {
    @Override
    public void runOpMode() {
        initializeHardware();
        // Step 1:  Drive forward for 1 second
        strafeRightMM(1500,200);
        closeRobot();
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
