
package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

@TeleOp(name="Topaz Teleop V4", group="Robot")
public class RobotTeleopTank_IterativeV4 extends OpMode {

    private Robot robot = new Robot();

    @Override
    public void init() {
        robot.init(hardwareMap,telemetry, false);
        //robot.loadRobotPosition(robot.hardwareMap.appContext);
        robot.loadRobotPosition();
        List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
        for (LLResultTypes.ColorResult cr : colorResults) {
            telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
        telemetry.addData("POSITION - Current X",  "%.2f", robot.positionTracker.getXPositionCM());
        telemetry.addData("POSITION - Current Y",  "%.2f", robot.positionTracker.getYPositionCM());
        telemetry.addData("POSITION - Current heading", robot.positionTracker.getHeadingDegrees());
        telemetry.addData(">", "Charlie v5 TELEOP is READY.  Press START.");    //
    }
    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        robot.loadRobotPosition();
    }

    @Override
    public void loop() {
        long currentTime = System.currentTimeMillis();

        robot.addTelemetry();

        // --- STOP & EMERGENCY ACTIONS
        if (gamepad2.back) {
            robot.emergencyReset();
        }

        //RESET THE POSITION (DO IN THE CORNER)
        if (gamepad1.back) {
            robot.positionTracker.resetPosition(0,0,0);
            //robot.saveRobotPosition(robot.hardwareMap.appContext);
            robot.saveRobotPosition();
        }

        if (gamepad1.y) {
            robot.specimenHook();
        }
        else {
            robot.specimenHookState = Robot.HookState.IDLE; // Transition to next step
        }

        if (gamepad1.x) {
            robot.sampleRelease();
        }
        else {
            robot.sampleReleaseState = Robot.releaseSampleFirstBucketState.IDLE; // Transition to next step
        }

        if (gamepad1.a) {
            robot.samplePickupGround();
        }
        else {
            robot.samplePickupState = Robot.pickupSampleGroundState.IDLE;

        }
        if (gamepad2.a) {
            robot.samplePickupGroundArmOnly();
        }
        else {
            robot.samplePickupArmOnlyState = Robot.pickupSampleGroundArmOnlyState.IDLE;
        }

        if (gamepad2.y) {
            robot.specimenHookArmOnly();
        }
        else {
            robot.specHookArmOnlyState = Robot.specimenHookArmOnlyState.IDLE;
        }

        if (gamepad2.x) {
            robot.sampleBucketArmOnly();
        }
        else {
            robot.sampleBucketArmOnlyState = Robot.releaseSampleFirstBucketArmOnlyState.IDLE;
        }

        if (!robot.isActionRunning()) {
            robot.driveWheels(-gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, true, false);
            robot.moveIntake(gamepad2.left_bumper, gamepad2.right_bumper);
            robot.moveArm(gamepad2.left_stick_y);
            robot.moveExtension(gamepad2.right_stick_y);
//            robot.moveWrist(gamepad2.left_bumper,gamepad2.right_bumper);
        }
        robot.updateTelemetry();
    }

}

