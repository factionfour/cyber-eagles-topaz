
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Topaz Teleop V4 TEST PID", group="Robot")
public class RobotTeleopTank_IterativeV4TESTPID extends OpMode {

    private Robot robot = new Robot();

    @Override
    public void init() {
        robot.init(hardwareMap,telemetry, false);
        telemetry.addData(">", "Charlie 3 is READY.  Press START.");    //
    }
    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        robot.positionTracker.loadRobotPosition(robot.hardwareMap.appContext);
    }

    @Override
    public void loop() {
        long currentTime = System.currentTimeMillis();
boolean actionRunning = false;
        robot.addTelemetry();

        // --- STOP & EMERGENCY ACTIONS
        if (gamepad2.back) {
            robot.emergencyReset();
        }

        //RESET THE POSITION (DO IN THE CORNER)
        if (gamepad1.back) {
            robot.positionTracker.resetPosition(0,0,0);
            robot.positionTracker.saveRobotPosition(robot.hardwareMap.appContext);
        }
//
//        if (gamepad1.y) {
//            robot.specimenHook();
//        }
//        else {
//            robot.specimenHookState = Robot.HookState.IDLE; // Transition to next step
//        }
//
//        if (gamepad1.x) {
//            robot.sampleRelease();
//        }
//        else {
//            robot.sampleReleaseState = Robot.releaseSampleFirstBucketState.IDLE; // Transition to next step
//        }
//
//        if (gamepad1.a) {
//            robot.samplePickupGround();
//        }
//        else {
//            robot.samplePickupState = Robot.pickupSampleGroundState.IDLE;
//
//        }
//        if (gamepad2.a) {
//            robot.samplePickupGroundArmOnly();
//        }
//        else {
//            robot.samplePickupArmOnlyState = Robot.pickupSampleGroundArmOnlyState.IDLE;
//        }
//
//        if (gamepad2.y) {
//            robot.sampleHookGroundArmOnly();
//        }
//        else {
//            robot.sampleHookArmOnlyState = Robot.sampleHookGroundArmOnlyState.IDLE;
//        }
//
//        if (gamepad2.x) {
//            robot.sampleBucketGroundArmOnly();
//        }
//        else {
//            robot.sampleBucketArmOnlyState = Robot.sampleBucketGroundArmOnlyState.IDLE;
//        }

        if (gamepad1.dpad_left) {
            actionRunning = true;
            robot.driveToPosition(0,100,0,false);
        }

        if (gamepad1.dpad_right) {
            actionRunning = true;
            robot.driveToPosition(0,0,0,false);
        }

        if (gamepad1.dpad_up) {
            actionRunning = true;
            robot.driveToPosition(100,0,0,false);
        }

        if (gamepad1.dpad_down) {
            actionRunning = true;
            robot.driveToPosition(0,0,0,false);
        }

        if (actionRunning) {
            robot.forceActionRunning(true);
        }
        else {
            robot.forceActionRunning(false);
        }
        if (!robot.isActionRunning()) {
            robot.driveWheels(-gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, true, false);
            robot.moveIntake(gamepad2.left_bumper, gamepad2.right_bumper);
            robot.moveArm(gamepad2.left_stick_y);
            robot.moveExtension(gamepad2.right_stick_y);
        }
        robot.updateTelemetry();
    }

}

