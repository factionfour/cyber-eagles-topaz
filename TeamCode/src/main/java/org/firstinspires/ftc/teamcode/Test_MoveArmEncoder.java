
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Z: Test Move arm Encoder (multi-step)", group="Robot")
public class Test_MoveArmEncoder extends AutoBase {
    @Override
    public void runOpMode() {
        initializeHardware();
        setInitialPosition();
        // Step 1:  Drive forward for 1 second
        moveArmEncoder(HOOK_ARM_HEIGHT,1500,400);
        moveExtension(HOOK_EXTENSION_POSITION,1500,200);
        moveArmEncoder(HOOK_RELEASE_ARM_HEIGHT,1500,200);
        moveExtension(0,2000,200);
        closeRobot();
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
