package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.concurrent.Callable;


public abstract class AutoBase4 extends LinearOpMode {

    public Robot robot;

    public void initializeHardware(double startXCM, double startYCM) {
        robot = new Robot();
        robot.init(hardwareMap,telemetry);
        robot.positionTracker.resetPosition(startXCM,startYCM);
        telemetry.addData(">", "Charlie 4 is READY.  Press START.");    //
        waitForStart();
    }

    //shut down the robot (end of auto)
    public void closeRobot() {
       //TODO
    }


    // Action 1: Example method to drive to a position
    public Callable<Boolean> driveToPositionAction(double targetXCM, double targetYCM, double targetHeadingDegrees) {
        return new Callable<Boolean>() {
            @Override
            public Boolean call() {
                return robot.driveToPosition(targetXCM, targetYCM, targetHeadingDegrees);  // This method returns true when complete
            }
        };
    }

    // Action 2: Example method to control a different action
    public Callable<Boolean> moveArmEncoderAction(int startPosition, int targetPosition) {
        return new Callable<Boolean>() {
            @Override
            public Boolean call() {
                // Call another method that returns true when complete
                return robot.moveArmEncoder(startPosition, targetPosition);
            }
        };
    }

    // Action 2: Example method to control a different action
    public Callable<Boolean> moveExtensionEncoderAction(int startPosition, int targetPosition) {
        return new Callable<Boolean>() {
            @Override
            public Boolean call() {
                // Call another method that returns true when complete
                return robot.moveExtensionEncoder(startPosition, targetPosition);
            }
        };
    }



    public boolean performActionsWithDelays(String actionTitle,
                                            Callable<Boolean> firstAction, long delay1,
                                            Callable<Boolean> secondAction, long delay2,
                                            Callable<Boolean> thirdAction, long delay3,
                                            Callable<Boolean> fourthAction, long delay4,
                                            Callable<Boolean> fifthAction) {

        Callable<Boolean>[] actions = new Callable[]{firstAction, secondAction, thirdAction, fourthAction, fifthAction};
        long[] delays = new long[]{0, delay1, delay2, delay3, delay4}; // Include initial delay as 0
        boolean[] actionCompleted = new boolean[5]; // Track completion of actions
        long startTime = System.currentTimeMillis(); // Start timer for delays

        while (opModeIsActive()) {
            long elapsedTime = System.currentTimeMillis() - startTime;

            for (int i = 0; i < actions.length; i++) {
                // Check if delay has passed and action is not yet completed
                if (!actionCompleted[i] && elapsedTime >= sumDelays(delays, i)) {
                    try {
                        if (actions[i] != null && actions[i].call()) {
                            actionCompleted[i] = true; // Mark as complete if call() returns true
                        }
                    } catch (Exception e) {
                        telemetry.addData("Error", "Exception occurred while executing action " + (i + 1));
                        e.printStackTrace();
                        actionCompleted[i] = true; // Treat it as complete to avoid blocking
                    }
                }
            }

            // Check if all actions are complete
            boolean allActionsCompleted = true;
            for (boolean completed : actionCompleted) {
                if (!completed) {
                    allActionsCompleted = false;
                    break;
                }
            }
            if (allActionsCompleted) {
                return true; // Exit when all actions have been completed
            }

            // Update telemetry for debugging
            telemetry.addData("RUNNING ACTION", actionTitle);
            telemetry.addData("Elapsed Time", elapsedTime);
            for (int i = 0; i < actionCompleted.length; i++) {
                telemetry.addData("Action " + (i + 1) + " Complete", actionCompleted[i]);
            }
            telemetry.update();
        }

        return false; // Return false if opMode ends prematurely
    }

    private long sumDelays(long[] delays, int index) {
        long total = 0;
        for (int i = 0; i <= index; i++) {
            total += delays[i];
        }
        return total;
    }

//
//    public boolean performActionsWithDelays( String actionTitle,
//            Action firstAction, long delay1,
//            Action secondAction, long delay2,
//            Action thirdAction, long delay3,
//            Action fourthAction, long delay4,
//            Action fifthAction
//    ) {
//        long actionStartTime = System.currentTimeMillis();
//        boolean[] actionCompleted = new boolean[5];  // Track if actions have been executed
//
//        // Execute first action if not null
//        if (firstAction != null) {
//            firstAction.execute();
//            actionCompleted[0] = true; // Mark first action as complete
//        } else {
//            actionCompleted[0] = true; // Consider first action completed if null
//        }
//
//        // Loop to monitor delays and execute subsequent actions
//        while (opModeIsActive()) {
//            long elapsedTime = System.currentTimeMillis() - actionStartTime;
//
//            // Execute second action after delay1 if not null
//            if (!actionCompleted[1] && elapsedTime >= delay1) {
//                if (secondAction != null) {
//                    secondAction.execute();
//                }
//                actionCompleted[1] = true; // Mark second action as complete, whether it was executed or not
//            }
//
//            // Execute third action after delay1 + delay2 if not null
//            if (!actionCompleted[2] && elapsedTime >= delay1 + delay2) {
//                if (thirdAction != null) {
//                    thirdAction.execute();
//                }
//                actionCompleted[2] = true; // Mark third action as complete, whether it was executed or not
//            }
//
//            // Execute fourth action after delay1 + delay2 + delay3 if not null
//            if (!actionCompleted[3] && elapsedTime >= delay1 + delay2 + delay3) {
//                if (fourthAction != null) {
//                    fourthAction.execute();
//                }
//                actionCompleted[3] = true; // Mark fourth action as complete, whether it was executed or not
//            }
//
//            // Execute fifth action after delay1 + delay2 + delay3 + delay4 if not null
//            if (!actionCompleted[4] && elapsedTime >= delay1 + delay2 + delay3 + delay4) {
//                if (fifthAction != null) {
//                    fifthAction.execute();
//                }
//                actionCompleted[4] = true; // Mark fifth action as complete, whether it was executed or not
//            }
//
//            // Check if all actions have been completed and delays have passed
//            if (elapsedTime >= (delay1 + delay2 + delay3 + delay4) &&
//                    actionCompleted[0] && actionCompleted[1] && actionCompleted[2] && actionCompleted[3] && actionCompleted[4]) {
//                return true;  // All actions completed
//            }
//
//            // Optionally, you can add telemetry for debugging or progress tracking
//
//            telemetry.addData("RUNNING ACTION", actionTitle);
//            telemetry.addData("Elapsed Time", elapsedTime);
//            telemetry.addData("Action 1 Complete", actionCompleted[0]);
//            telemetry.addData("Action 2 Complete", actionCompleted[1]);
//            telemetry.addData("Action 3 Complete", actionCompleted[2]);
//            telemetry.addData("Action 4 Complete", actionCompleted[3]);
//            telemetry.addData("Action 5 Complete", actionCompleted[4]);
//            telemetry.update();
//        }
//
//        return false;  // Return false if opMode ends prematurely before completion
//    }


//    public boolean performActionsWithDelays(
//            Action firstAction, long delay1,
//            Action secondAction, long delay2,
//            Action thirdAction, long delay3,
//            Action fourthAction, long delay4,
//            Action fifthAction
//    ) {
//        long actionStartTime = System.currentTimeMillis();
//
//        // Execute the first action if not null
//        if (firstAction != null) {
//            firstAction.execute();
//        }
//
//        // Loop to monitor delays and execute subsequent actions
//        while (opModeIsActive()) {
//            long elapsedTime = System.currentTimeMillis() - actionStartTime;
//
//            // Execute second action if time has passed and it is not null
//            if (elapsedTime >= delay1 && secondAction != null) {
//                secondAction.execute();
//                secondAction = null; // Prevent re-execution
//            }
//
//            // Execute third action if time has passed and it is not null
//            if (elapsedTime >= delay1 + delay2 && thirdAction != null) {
//                thirdAction.execute();
//                thirdAction = null; // Prevent re-execution
//            }
//
//            // Execute fourth action if time has passed and it is not null
//            if (elapsedTime >= delay1 + delay2 + delay3 && fourthAction != null) {
//                fourthAction.execute();
//                fourthAction = null; // Prevent re-execution
//            }
//
//            // Execute fifth action if time has passed and it is not null
//            if (elapsedTime >= delay1 + delay2 + delay3 + delay4 && fifthAction != null) {
//                fifthAction.execute();
//                fifthAction = null; // Prevent re-execution
//                return true; // All actions completed
//            }
//
//            // Add telemetry for debugging (optional)
//            telemetry.addData("Elapsed Time", elapsedTime);
//            telemetry.update();
//        }
//
//        return false; // Exit if opMode ends prematurely
//    }



}
