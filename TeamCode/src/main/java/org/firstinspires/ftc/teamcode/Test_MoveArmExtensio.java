
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name="Z: Test Move arm (multi-step)", group="Robot")
public class Test_MoveArmExtensio extends AutoBase {
    @Override
    public void runOpMode() {
        initializeHardware();
        setInitialPosition();
        // Step 1:  Drive forward for 1 second
        moveArm(HOOK_ARM_HEIGHT,3500,400);
        moveExtension(HOOK_EXTENSION_POSITION,3500,200);
        //moveArm(HOOK_RELEASE_ARM_HEIGHT,1500,200);
        //moveExtension(0,2000,200);
        closeRobot();
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
