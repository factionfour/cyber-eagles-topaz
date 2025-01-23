package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public abstract class AutoBase4 extends LinearOpMode {

    public Robot robot = new Robot();

    public void initRobot() {
        robot.init(hardwareMap,telemetry);
        telemetry.addData(">", "Charlie 3 is READY.  Press START.");    //
    }
    //shut down the robot (end of auto)
    public void closeRobot() {
       //TODO
    }

    public boolean performActionsWithDelays(
            Action firstAction, long delay1,
            Action secondAction, long delay2,
            Action thirdAction, long delay3,
            Action fourthAction, long delay4,
            Action fifthAction
    ) {
        long actionStartTime = System.currentTimeMillis();

        // Execute the first action if not null
        if (firstAction != null) {
            firstAction.execute();
        }

        // Loop to monitor delays and execute subsequent actions
        while (opModeIsActive()) {
            long elapsedTime = System.currentTimeMillis() - actionStartTime;

            // Execute second action if time has passed and it is not null
            if (elapsedTime >= delay1 && secondAction != null) {
                secondAction.execute();
                secondAction = null; // Prevent re-execution
            }

            // Execute third action if time has passed and it is not null
            if (elapsedTime >= delay1 + delay2 && thirdAction != null) {
                thirdAction.execute();
                thirdAction = null; // Prevent re-execution
            }

            // Execute fourth action if time has passed and it is not null
            if (elapsedTime >= delay1 + delay2 + delay3 && fourthAction != null) {
                fourthAction.execute();
                fourthAction = null; // Prevent re-execution
            }

            // Execute fifth action if time has passed and it is not null
            if (elapsedTime >= delay1 + delay2 + delay3 + delay4 && fifthAction != null) {
                fifthAction.execute();
                fifthAction = null; // Prevent re-execution
                return true; // All actions completed
            }

            // Add telemetry for debugging (optional)
            telemetry.addData("Elapsed Time", elapsedTime);
            telemetry.update();
        }

        return false; // Exit if opMode ends prematurely
    }



}
