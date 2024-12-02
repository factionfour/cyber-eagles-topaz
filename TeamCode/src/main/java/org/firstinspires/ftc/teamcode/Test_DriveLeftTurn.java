
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Z: Test Left turn 90", group="Robot")
public class Test_DriveLeftTurn extends AutoBase {
    @Override
    public void runOpMode() {
        initializeHardware();
        setInitialPosition();
        // Step 1:  Drive forward for 1 second
        turnLeft(90,200);
        closeRobot();
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
