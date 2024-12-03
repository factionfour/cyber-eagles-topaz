
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Topaz: Auto Drive Position Left", group="Robot")
public class RobotAutoLeft extends AutoBase {
    @Override
    public void runOpMode() {
        initializeHardware();
        
        setInitialPosition();


        closeRobot();
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
