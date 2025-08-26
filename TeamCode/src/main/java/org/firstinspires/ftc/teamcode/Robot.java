
package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;

import android.content.SharedPreferences;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileInputStream;
import java.io.IOException;
import java.security.KeyStore;
import java.util.ArrayList;
import java.util.List;

import javax.annotation.processing.Completion;


public class Robot {
    // define each motor, servo, imu and touch sensor
    public HardwareMap hardwareMap = null;
    private Telemetry telemetry;
    public DcMotor frontleftDrive = null;
    public DcMotor backleftDrive = null;
    public DcMotor backrightDrive = null;
    public DcMotor frontrightDrive = null;
    public DcMotor armMotor = null;
    public DcMotor extensionArmMotor = null;
    public Servo leftWheelServo = null;
    public Servo rightWheelServo = null;
    public IMU imu;
    public Servo Wrist = null;
    public TouchSensor touchsensor;

    double DRIVING_SPEEDFACTOR_HUMAN = 0.7;
    double DRIVING_SPEEDFACTOR_AUTO = 1;
    double TURNING_SPEEDFACTOR_AUTO = 0.9;
    double DRIVING_SPEEDFACTOR_SLOW_AUTO = 0.9;

    // Arm motor limits and power
    int ARM_MIN_POSITION = 140;//100;    // Minimum encoder position (fully Lowered// )
    int ARM_MAX_POSITION = 400; // Maximum encoder position (fully Raised)
    double ARM_BASE_POWER = 0.2;
    double ARM_EXTRA_FORCE = 0.01;
    double ARM_MAX_SPEED = 0.9;
    double ARM_MIN_SPEED = 0.3;
    double ARM_RAMP_TICKS = 50;
    double ARM_MANUAL_MAX_SPEED = 40;

    // Extension limits and power
    int EXTENSION_MIN_POSITION = 0;    // Minimum height (fully lowered)
    int EXTENSION_MAX_POSITION = 2700; // Maximum height (fully raised)
    double EXTENSION_BASE_POWER = 0.3;
    double EXTENSION_EXTRA_FORCE = 0.6;

    double EXTENSION_MIN_SPEED = 0.3; // How far to move per iterationfsampleRelease
    double EXTENSION_MAX_SPEED = 0.95; // How far to move per iteration
    double EXTENSION_RAMP_TICKS = 50;
    double EXTENSION_MANUAL_MAX_SPEED = 100;

    //tolernances
    int MOTOR_TOLERANCE = 5; // Acceptable error in encoder ticks
    double POSITION_TOLERANCE_CM = 20;
    double POSITION_ADJUST_TOLERANCE_CM = 1;
    double HEADING_TOLERANCE_DEGREES = 1;

    //servo limits
    double SERVO_STOPPED = 0.5;
    double SERVO_FORWARD = 1;
    double SERVO_BACKWARD = 0;

    //drive speeds
    double DRIVE_MAX_POWER = 1; // Maximum power
    double DRIVE_MIN_POWER = 0.3; // Minimum power to prevent stalling
    double DRIVE_SLOW_THRESHOLD = 20.0; // Distance (CM) where slowdown begins
    double DRIVE_CRAWL_THRESHOLD = 2.0; // Distance (CM) where slow crawl is enforced

    //turn speeds
    double TURN_MAX_POWER = 0.8; // Maximum turning power
    double TURN_MIN_POWER = 0.2; // Minimum turning power for precision
    double TURN_SLOWDOWN_THRESHOLD = Math.toRadians(10.0); // Angle threshold for starting to slow down

    // PID variables for forwards
    double Drive_Kd = 0.05; // Add some damping
    double Drive_Kp = 0.05; // Keep proportional gain
    double Drive_previousErrorY = 0;
    double Drive_previousErrorX = 0;

    // PID variables for turning
    double TURN_KD = 0.02;
    double TURN_KP = 0.9;
    double Turn_PrevDelta = 0;
    private int settlingCycles = 0;


    //Variables for lime pipelines


    //pre-defined positions
    int DRIVE_ARM_POSITION = 200;
    //Hook
    int HOOK_EXTENSION_POSITION = 1651;
    int HOOK_ARM_HEIGHT = 700;
    int HOOK_DEGREES = 0;
    int HOOK_POS_X = 58;
    int HOOK_POS_Y = 158;
    int HOOK_ARM_HEIGHT_2 = 565;//620;
    int POST_HOOK_POS_X = 40;
    int POST_HOOK_POS_Y = 158;
    //Pickup
    int PICKUP_SAMPLE_ARM_HEIGHT = 140;//135;//286;
    int PICKUP_SAMPLE_EXTENSION_POSITION = 1630;
    int PICKUP_SAMPLE_DEGREES = 180;

    int PICKUP_SAMPLE_POS_X = 50;
    int PICKUP_SAMPLE_POS_INTAKE_X = 38;
    int PICKUP_SAMPLE_POS_NOPICKUP_X = 70;
    int PICKUP_SAMPLE_POS_Y = 37;
    //release
    int RELEASE_SAMPLE_ARM_HEIGHT = 950;
    int RELEASE_SAMPLE_ARM_HEIGHT_2 = 850;
    int RELEASE_SAMPLE_EXTENSION_POSITION = 2820;//2700;
    int RELEASE_SAMPLE_DEGREES = 138;
    int RELEASE_SAMPLE_POS_X = 24;
    int RELEASE_SAMPLE_POS_Y = 298;
    //first block
    int PUSH_FIRST_BLOCK_POS_X_0 = 68;
    int PUSH_FIRST_BLOCK_POS_Y_0 = 64;

    int PUSH_FIRST_BLOCK_POS_X_1 = 132;
    int PUSH_FIRST_BLOCK_POS_Y_1 = 64;
    int PUSH_FIRST_BLOCK_POS_X_2 = 132;
    int PUSH_FIRST_BLOCK_POS_Y_2 = 40;
    int PUSH_FIRST_BLOCK_POS_X_3 = 10;
    int PUSH_FIRST_BLOCK_POS_Y_3 = 40;
    int PUSH_FIRST_BLOCK_POS_X_4 = 132;
    int PUSH_FIRST_BLOCK_POS_Y_4 = 40;
    //second block
    int PUSH_SECOND_BLOCK_POS_X_1 = 132;
    int PUSH_SECOND_BLOCK_POS_Y_1 = 15;
    int PUSH_SECOND_BLOCK_POS_X_2 = 22;
    int PUSH_SECOND_BLOCK_POS_Y_2 = 15;
    int PUSH_SECOND_BLOCK_POS_X_3 = 56;
    int PUSH_SECOND_BLOCK_POS_Y_3 = 15;
    int PUSH_SECOND_BLOCK_POS_X_4 = 10;

    int PICKUP_BLOCK_POS_X = 35;
    int PICKUP_BLOCK_POS_Y = 276;
    int PICKUP_BLOCK_2_POS_Y = 300;
    int PICKUP_BLOCK_POS_INTAKE_X = 50;

    int PARK_LEFT_AUTO_POS_1_X = 125;
    int PARK_LEFT_AUTO_POS_1_Y = 257;

    int PARK_LEFT_AUTO_POS_2_X = 130;
    int PARK_LEFT_AUTO_POS_2_Y = 230;
    int PARK_LEFT_AUTO_POS_2_HEADING = -90;
    int PARK_ARM_POSITION = 700;
    int PARK_ARM_POSITION_2 = 617;
    int PARK_EXTENSION_POSITION = 1050;

    int dynamicArmMinPosition = 0;
    double currentExtensionPower = 0;
    double currentArmPower = 0;
    int extensionTargetPosition = 0;
    int armTargetPosition = 0;

    double currentForward;
    double currentTurn;
    double currentStrafe;


    public manualArmState tmpArmState = manualArmState.IDLE;
    public manualExtensionState tmpExtensionState = manualExtensionState.IDLE;
    public HookState specimenHookState = HookState.IDLE;
    //public HookReleaseState specimenReleaseState = HookReleaseState.IDLE;
    public pickupSampleGroundArmOnlyState samplePickupArmOnlyState = pickupSampleGroundArmOnlyState.IDLE;
    public releaseSampleFirstBucketState sampleReleaseState = releaseSampleFirstBucketState.IDLE;
    public pickupSampleGroundState samplePickupState = pickupSampleGroundState.IDLE;

    public specimenHookArmOnlyState specHookArmOnlyState = specimenHookArmOnlyState.IDLE;
    public releaseSampleFirstBucketArmOnlyState sampleBucketArmOnlyState = releaseSampleFirstBucketArmOnlyState.IDLE;


    public manualServoState tmpServoState = manualServoState.IDLE;
    public driveToPositionState tmpDriveState = driveToPositionState.IDLE;
    public wristServoState tempServoState = wristServoState.IDLE;

    int tmpArmPositionHolder = 0;
    int tmpExtensionPositionHolder = 0;
    long tmpActionStartTime = 0;
    public RobotPositionTracker positionTracker;
    boolean sampleCaptured = false;
    private ElapsedTime intakeTimer = null;

    //Limelight var
    boolean autoDriveOn = false;
    private Limelight3A limelight;
    LLResult result = limelight.getLatestResult();

    double LimeX = result.getTx();
    double LimeY = result.getTy();
    double LimeA = result.getTa();
    int CURRENT_PIPELINE = 0;
    private long lastSeenTimeLime = 0;
    int LimeStep = 0;


    public void init(HardwareMap hwMap, Telemetry telem, boolean resetArm) {
        hardwareMap = hwMap;
        telemetry = telem;
        // Define and Initialize wheel Motors
        frontleftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontrightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backrightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        backleftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");

        //limelight hardware map
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        LLResult result = limelight.getLatestResult();

        // Set directions to each Motor
        frontleftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontrightDrive.setDirection(DcMotor.Direction.REVERSE);
        backrightDrive.setDirection(DcMotor.Direction.REVERSE);
        backleftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize arm motor
        armMotor = hardwareMap.get(DcMotor.class, "arm_1");
        if (resetArm) {
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //armTargetPosition = ARM_MIN_POSITION;

        // Initialize extension arm motor
        extensionArmMotor = hardwareMap.get(DcMotor.class, "arm_extendo");
        extensionArmMotor.setDirection(DcMotor.Direction.REVERSE);
        if (resetArm) {
            extensionArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        extensionArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize wheel servos
        leftWheelServo = hardwareMap.get(Servo.class, "servo_one");
        rightWheelServo = hardwareMap.get(Servo.class, "servo_two");
        Wrist = hardwareMap.get(Servo.class, "wrist");

        leftWheelServo.setPosition(0.5); // Neutral position
        rightWheelServo.setPosition(0.5); // Neutral position
        Wrist.setPosition(0.5);

        touchsensor = hardwareMap.get(TouchSensor.class, "touch sensor");
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize((new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.LEFT))));

        positionTracker = new RobotPositionTracker(hardwareMap.get(GoBildaPinpointDriver.class, "odo"), hardwareMap.get(IMU.class, "imu"));


    }

    //  Telemetry
    public void addTelemetry() {
        LLStatus status = limelight.getStatus();
        LLResult result = limelight.getLatestResult();
        CURRENT_PIPELINE = limelight.getLatestResult().getPipelineIndex();

//        //telemetry.addData("Button pressed", touchsensor.isPressed());
////        telemetry.addData("front",  "%.2f", currentForward);
////        telemetry.addData("turn", "%.2f", currentTurn);
////        telemetry.addData("strafe", "%.2f", currentStrafe);
        telemetry.addData("POSITION - Current X", "%.2f", positionTracker.getXPositionCM());
        telemetry.addData("POSITION - Current Y", "%.2f", positionTracker.getYPositionCM());
        telemetry.addData("POSITION - Current heading", positionTracker.getHeadingDegrees());
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d", status.getTemp(), status.getCpu(), (int) status.getFps());
        telemetry.addData("Pipeline", "Index: %d, Type: %s", status.getPipelineIndex(), status.getPipelineType());
        telemetry.addData("EXTENSION - Current Position", extensionArmMotor.getCurrentPosition());
//        telemetry.addData("EXTENSION - Target Position", extensionTargetPosition);
//        //telemetry.addData("Extension Calculated Power", currentExtensionPower);
//        //telemetry.addData("Extension Motor Busy", extensionArmMotor.isBusy());
//
        telemetry.addData("ARM - Current Position", armMotor.getCurrentPosition());
//        telemetry.addData("ARM - Target Position", armTargetPosition);
//        //telemetry.addData("Arm Calculated Min Position", dynamicArmMinPosition);
//        //telemetry.addData("Arm Calculated Power", currentArmPower);
//
//        //telemetry.addData("Left Servo Position", leftWheelServo.getPosition());
//        //telemetry.addData("Right Servo Position", rightWheelServo.getPosition());*/

        List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
        for (LLResultTypes.ColorResult cr : colorResults) {
            telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
        }
//
    }

    public void updateTelemetry() {
        telemetry.update();
    }

    public void driveWheels(double tmpForward, double tmpTurn, double tmpStrafe, boolean human, boolean slow) {
        currentForward = tmpForward;
        currentTurn = tmpTurn;
        currentStrafe = tmpStrafe;

        // Apply dead zone and smoothing
        if (human) {
            currentForward = Math.abs(currentForward) > 0.05 ? Math.pow(currentForward, 3) : 0.0;
            currentTurn = Math.abs(currentTurn) > 0.05 ? currentTurn : 0.0; // Keep turn linear
            currentStrafe = Math.abs(currentStrafe) > 0.05 ? Math.pow(currentStrafe, 3) : 0.0;
        } else {
            // Small dead zone in auto mode to prevent small jitters
            if (Math.abs(currentForward) < 0.02) currentForward = 0.0;
            if (Math.abs(currentTurn) < 0.02) currentTurn = 0.0;
            if (Math.abs(currentStrafe) < 0.02) currentStrafe = 0.0;
        }

        // Combine inputs for omnidirectional control (keeping your sign conventions)
        double frontLeftPower = -currentForward + -currentTurn + -currentStrafe;
        double frontRightPower = -currentForward - -currentTurn - -currentStrafe;
        double backLeftPower = currentForward + -currentTurn - -currentStrafe;
        double backRightPower = currentForward - -currentTurn + -currentStrafe;

        // Normalize power values to prevent exceeding max motor power
        double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        // Apply speed reduction for different driving modes
        double speedFactor = 1.0; // Default to full speed
        if (human) {
            speedFactor = DRIVING_SPEEDFACTOR_HUMAN;
        } else if (slow) {
            speedFactor = DRIVING_SPEEDFACTOR_SLOW_AUTO; // Extra slow for precision
        } else if (currentTurn == 0) {
            speedFactor = DRIVING_SPEEDFACTOR_AUTO; // Normal slow movement
        } else {
            speedFactor = TURNING_SPEEDFACTOR_AUTO; // Slow down turning
        }

        // Apply the speed factor
        frontLeftPower *= speedFactor;
        frontRightPower *= speedFactor;
        backLeftPower *= speedFactor;
        backRightPower *= speedFactor;


        // Set motor powers
        frontleftDrive.setPower(frontLeftPower);
        frontrightDrive.setPower(frontRightPower);
        backleftDrive.setPower(backLeftPower);
        backrightDrive.setPower(backRightPower);

        if (currentForward > 0) telemetry.addData("DRIVE POWER - forward", currentForward);
        if (currentTurn > 0) telemetry.addData("DRIVE POWER - Turn", currentTurn);
        if (currentStrafe > 0) telemetry.addData("DRIVE POWER - Strafe", currentStrafe);
        if (frontLeftPower > 0) telemetry.addData("WHEEL POWER - frontLeft", frontLeftPower);
        if (frontRightPower > 0) telemetry.addData("WHEEL POWER - frontRight", frontRightPower);
        if (backLeftPower > 0) telemetry.addData("WHEEL POWER - backLeft", backLeftPower);
        if (backRightPower > 0) telemetry.addData("WHEEL POWER - backRight", backRightPower);

        long currentTime = System.currentTimeMillis();
        positionTracker.updatePosition();
    }

    public boolean driveToPosition(double targetXCM, double targetYCM, double targetHeadingDegrees, boolean driveSlow) {
        if (tmpDriveState == driveToPositionState.IDLE) {
            tmpDriveState = driveToPositionState.DRIVE;
            // Reset PID variables on start of new movement
            Drive_previousErrorX = 0;
            Drive_previousErrorY = 0;
            Turn_PrevDelta = 0;
            lastTurnVelocity = 0;
            turnIntegralError = 0;
            turnVelocityHistory.clear();  // Clear velocity history
            for (int i = 0; i < VELOCITY_HISTORY_SIZE; i++) {
                turnVelocityHistory.add(0.0);  // Initialize with zeros
            }
            lastUpdateTime = System.currentTimeMillis() / 1000.0; // Time in seconds
        }

        // Convert targetHeading from degrees to radians
        double targetHeadingRadians = Math.toRadians(targetHeadingDegrees);

        // Update the robot's current position and heading
        positionTracker.updatePosition();
        double currentX = positionTracker.getXPositionCM();
        double currentY = positionTracker.getYPositionCM();
        double currentHeading = positionTracker.getHeading();

        // Calculate current time and delta time since last update
        double currentTime = System.currentTimeMillis() / 1000.0; // Time in seconds
        double deltaTime = currentTime - lastUpdateTime;
        lastUpdateTime = currentTime;

        // Prevent division by zero and handle first iteration
        if (deltaTime <= 0 || deltaTime > 0.1) { // Cap delta time to prevent huge values
            deltaTime = 0.02; // Default to 20ms
        }

        // Calculate heading velocity (how fast we're turning) with better filtering
        double currentTurnVelocity = 0;
        if (deltaTime > 0) {
            double rawVelocity = (currentHeading - previousHeading) / deltaTime;

            // Handle heading wrap-around for velocity calculation
            if (Math.abs(rawVelocity) > Math.PI) {
                if (rawVelocity > 0) {
                    rawVelocity -= 2 * Math.PI / deltaTime;
                } else {
                    rawVelocity += 2 * Math.PI / deltaTime;
                }
            }

            currentTurnVelocity = rawVelocity;
        }
        previousHeading = currentHeading;

        // Store velocity in history buffer for smoothing
        turnVelocityHistory.add(0, currentTurnVelocity);  // Add to front
        if (turnVelocityHistory.size() > VELOCITY_HISTORY_SIZE) {
            turnVelocityHistory.remove(turnVelocityHistory.size() - 1);  // Remove oldest
        }

        // Calculate smoothed velocity (moving average) with outlier rejection
        double smoothedVelocity = 0;
        int validSamples = 0;
        double velocitySum = 0;

        // First pass: calculate mean
        for (Double v : turnVelocityHistory) {
            velocitySum += v;
            validSamples++;
        }
        double meanVelocity = validSamples > 0 ? velocitySum / validSamples : 0;

        // Use the smoothed velocity for control
        double turnVelocity = meanVelocity;

        // Calculate differences to target
        double deltaX = targetXCM - currentX;
        double deltaY = targetYCM - currentY;
        double distanceToTarget = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        double deltaHeading = targetHeadingRadians - currentHeading;

        // Normalize deltaHeading to the range [-π, π]
        if (deltaHeading > Math.PI) deltaHeading -= 2 * Math.PI;
        if (deltaHeading < -Math.PI) deltaHeading += 2 * Math.PI;

        // Log telemetry data
        telemetry.addData("DRIVE STATE", tmpDriveState);
        telemetry.addData("POSITION - Current X", "%.2f", currentX);
        telemetry.addData("POSITION - Current Y", "%.2f", currentY);
        telemetry.addData("POSITION - Current heading", Math.toDegrees(currentHeading));
        telemetry.addData("TARGET POS - X", targetXCM);
        telemetry.addData("TARGET POS - Y", targetYCM);
        telemetry.addData("DISTANCE TO TARGET", distanceToTarget);
        telemetry.addData("TARGET HEADING DELTA", Math.toDegrees(deltaHeading));
        telemetry.addData("TURN VELOCITY (deg/s)", Math.toDegrees(currentTurnVelocity));
        telemetry.addData("SMOOTHED VELOCITY (deg/s)", Math.toDegrees(turnVelocity));


        // Step 1: Combined drive and turn movement
        if ((distanceToTarget > POSITION_TOLERANCE_CM || Math.abs(deltaHeading) > Math.toRadians(HEADING_TOLERANCE_DEGREES)) && tmpDriveState == driveToPositionState.DRIVE) {
            // Calculate angle to target in global coordinates
            double angleToTarget = Math.atan2(deltaY, deltaX);
            double relativeAngleToTarget = angleToTarget - currentHeading;

            // Normalize the relative angle
            if (relativeAngleToTarget > Math.PI) relativeAngleToTarget -= 2 * Math.PI;
            if (relativeAngleToTarget < -Math.PI) relativeAngleToTarget += 2 * Math.PI;

            // Transform target position into robot's coordinate frame
            double robotRelativeX = distanceToTarget * Math.cos(relativeAngleToTarget);
            double robotRelativeY = distanceToTarget * Math.sin(relativeAngleToTarget);

            // Calculate drive powers using PID
            double derivativeX = robotRelativeX - Drive_previousErrorX;
            double derivativeY = robotRelativeY - Drive_previousErrorY;
            double forwardPower = (Drive_Kp * robotRelativeX) + (Drive_Kd * derivativeX);
            double strafePower = (Drive_Kp * robotRelativeY) + (Drive_Kd * derivativeY);

            // Store previous errors
            Drive_previousErrorX = robotRelativeX;
            Drive_previousErrorY = robotRelativeY;

            // *** IMPROVED TURNING CONTROL WITH BETTER DEADBAND ***

            // Apply PID with anti-windup (small integral term to eliminate steady-state error)
            double TURN_KI = 0.005; // Reduced integral gain

            // Only apply integral when close to target and moving slowly
            if (Math.abs(deltaHeading) < Math.toRadians(10.0) && Math.abs(turnVelocity) < Math.toRadians(15.0)) {
                turnIntegralError += deltaHeading * deltaTime;
                // Anti-windup: limit integral term more aggressively
                turnIntegralError = Math.max(Math.min(turnIntegralError, MAX_INTEGRAL_ERROR * 0.5), -MAX_INTEGRAL_ERROR * 0.5);
            } else {
                turnIntegralError = 0; // Reset integral when far from target or moving fast
            }

            // Calculate turn derivative (rate of change of error)
            double turnDerivative = 0;
            if (deltaTime > 0) {
                turnDerivative = (deltaHeading - Turn_PrevDelta) / deltaTime;
            }
            Turn_PrevDelta = deltaHeading;

            // Calculate turn power using PID
            double turnPower = 0;

            // IMPROVED DEADBAND: Larger deadband with velocity consideration
            double POSITION_DEADBAND = Math.toRadians(2.0); // 2 degrees position deadband
            double VELOCITY_DEADBAND = Math.toRadians(5.0); // 5 degrees/second velocity deadband

            // Check if we're close enough to target AND moving slowly enough
            boolean inPositionDeadband = Math.abs(deltaHeading) < POSITION_DEADBAND;
            boolean inVelocityDeadband = Math.abs(turnVelocity) < VELOCITY_DEADBAND;

            if (inPositionDeadband && inVelocityDeadband) {
                // We're close and moving slowly - stop turning
                turnPower = 0;
                telemetry.addData("TURN STATUS", "IN DEADBAND - STOPPED");
            } else if (inPositionDeadband && !inVelocityDeadband) {
                // We're close but still moving - apply light damping
                turnPower = -0.2 * Math.signum(turnVelocity);
                telemetry.addData("TURN STATUS", "CLOSE - DAMPING");
            } else {
                // Normal PID control
                turnPower = (TURN_KP * deltaHeading) +
                        (TURN_KD * turnDerivative) +
                        (TURN_KI * turnIntegralError);

                // Apply progressive slowdown as we approach target
                if (Math.abs(deltaHeading) < TURN_SLOWDOWN_THRESHOLD) {
                    double slowdownFactor = Math.abs(deltaHeading) / TURN_SLOWDOWN_THRESHOLD;
                    slowdownFactor = Math.max(slowdownFactor, 0.3); // Minimum 30% power
                    turnPower *= slowdownFactor;
                    telemetry.addData("SLOWDOWN FACTOR", slowdownFactor);
                }

                telemetry.addData("TURN STATUS", "NORMAL PID");
            }

            // Limit turn power
            turnPower = Math.min(Math.max(-TURN_MAX_POWER, turnPower), TURN_MAX_POWER);

            // Set minimum power threshold only when not in deadband
            if (!inPositionDeadband && Math.abs(turnPower) < TURN_MIN_POWER && Math.abs(turnPower) > 0) {
                turnPower = TURN_MIN_POWER * Math.signum(turnPower);
            }

            // Log the final turn power
            telemetry.addData("Final Turn Power", turnPower);
            telemetry.addData("Position Deadband", inPositionDeadband);
            telemetry.addData("Velocity Deadband", inVelocityDeadband);

            // Normalize drive powers
            double maxDrivePower = Math.max(Math.abs(forwardPower), Math.abs(strafePower));
            if (maxDrivePower > DRIVE_MAX_POWER) {
                forwardPower = (forwardPower / maxDrivePower) * DRIVE_MAX_POWER;
                strafePower = (strafePower / maxDrivePower) * DRIVE_MAX_POWER;
            }

            // Send powers to the drive system
            driveWheels(forwardPower, -turnPower, -strafePower, false, driveSlow);

        } else if (tmpDriveState == driveToPositionState.DRIVE) {
            driveWheels(0, 0, 0, false, driveSlow);

            tmpDriveState = driveToPositionState.SETTLING;
            settlingCycles = 0;  // Reset settling counter
        }

        // Add settling state handler with shorter settling time
        if (tmpDriveState == driveToPositionState.SETTLING) {
            driveWheels(0, 0, 0, false, driveSlow);
            settlingCycles++;

            if (settlingCycles >= 3) {  // Reduced from 5 to 3 cycles
                // Get latest position after settling
                positionTracker.updatePosition();
                deltaX = targetXCM - positionTracker.getXPositionCM();
                deltaY = targetYCM - positionTracker.getYPositionCM();

                if (Math.abs(deltaX) > POSITION_ADJUST_TOLERANCE_CM ||
                        Math.abs(deltaY) > POSITION_ADJUST_TOLERANCE_CM) {
                    tmpDriveState = driveToPositionState.ADJUST;
                } else {
                    tmpDriveState = driveToPositionState.COMPLETE;
                }
            }
        }

        // Step 2: Final position adjustment phase - SIMPLIFIED
        if (tmpDriveState == driveToPositionState.ADJUST) {
            // Update position tracking
            positionTracker.updatePosition();
            deltaX = targetXCM - positionTracker.getXPositionCM();
            deltaY = targetYCM - positionTracker.getYPositionCM();
            currentHeading = positionTracker.getHeading();

            // Check heading first with simpler control
            deltaHeading = targetHeadingRadians - currentHeading;
            if (deltaHeading > Math.PI) deltaHeading -= 2 * Math.PI;
            if (deltaHeading < -Math.PI) deltaHeading += 2 * Math.PI;

            if (Math.abs(deltaHeading) > Math.toRadians(HEADING_TOLERANCE_DEGREES)) {
                // Simple proportional control for adjustment phase
                double turnPower = Math.signum(deltaHeading) * TURN_MIN_POWER * 0.7;
                driveWheels(0, -turnPower, 0, false, false);
            }
            // Only proceed with X/Y adjustments if heading is correct
            else if (Math.abs(deltaX) > POSITION_ADJUST_TOLERANCE_CM || Math.abs(deltaY) > POSITION_ADJUST_TOLERANCE_CM) {
                // Calculate angle to target in global coordinates
                double angleToTarget = Math.atan2(deltaY, deltaX);
                double relativeAngleToTarget = angleToTarget - currentHeading;

                // Normalize the relative angle
                if (relativeAngleToTarget > Math.PI) relativeAngleToTarget -= 2 * Math.PI;
                if (relativeAngleToTarget < -Math.PI) relativeAngleToTarget += 2 * Math.PI;

                // Transform into robot's coordinate frame
                distanceToTarget = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
                double robotRelativeX = distanceToTarget * Math.cos(relativeAngleToTarget);
                double robotRelativeY = distanceToTarget * Math.sin(relativeAngleToTarget);

                // Move in one direction at a time with reduced power
                if (Math.abs(robotRelativeX) > Math.abs(robotRelativeY)) {
                    // Move only in the X direction
                    driveWheels(Math.signum(robotRelativeX) * DRIVE_MIN_POWER * 0.6, 0, 0, false, driveSlow);
                } else {
                    // Move only in the Y direction
                    driveWheels(0, 0, -(Math.signum(robotRelativeY) * DRIVE_MIN_POWER * 0.6), false, driveSlow);
                }
            } else {
                driveWheels(0, 0, 0, false, driveSlow);
                tmpDriveState = driveToPositionState.COMPLETE;
            }
        }

        telemetry.addData("Drive State", tmpDriveState);
        return tmpDriveState == driveToPositionState.COMPLETE;
    }

    // You'll need to add these class variables:
    private double lastUpdateTime = 0;
    private double previousHeading = 0;
    private double lastTurnVelocity = 0;
    private double turnIntegralError = 0;
    private static final double MAX_INTEGRAL_ERROR = Math.toRadians(10.0); // Reduced limit
    private static final double MAX_TURN_DECELERATION = Math.toRadians(90.0); // radians/sec^2
    private static final int VELOCITY_HISTORY_SIZE = 3;  // Reduced from 5 to 3
    private ArrayList<Double> turnVelocityHistory = new ArrayList<>(VELOCITY_HISTORY_SIZE);
//
//    public boolean driveToPosition(double targetXCM, double targetYCM, double targetHeadingDegrees, boolean driveSlow) {
//        if (tmpDriveState == driveToPositionState.IDLE) {
//            tmpDriveState = driveToPositionState.DRIVE;
//            //tmpDriveState = driveToPositionState.TURN;
//        }
//        // Convert targetHeading from degrees to radians
//        double targetHeadingRadians = Math.toRadians(targetHeadingDegrees);
//
//        // Update the robot's current position and heading
//        positionTracker.updatePosition();
//        double currentX = positionTracker.getXPositionCM();
//        double currentY = positionTracker.getYPositionCM();
//        double currentHeading = positionTracker.getHeading();
//
//        // Calculate differences to target
//        double deltaX = targetXCM - currentX;
//        double deltaY = targetYCM - currentY;
//        double distanceToTarget = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
//        double deltaHeading = targetHeadingRadians - currentHeading;
//
//        // Normalize deltaHeading to the range [-π, π] to ensure shortest turn
//        if (deltaHeading > Math.PI) deltaHeading -= 2 * Math.PI;
//        if (deltaHeading < -Math.PI) deltaHeading += 2 * Math.PI;
//
//        if (Math.abs(deltaHeading) < Math.toRadians(HEADING_TOLERANCE_DEGREES)) {
//            deltaHeading = 0; // Snap to 0 to avoid unnecessary small rotations
//        }
//
//        telemetry.addData("DRIVE STATE", tmpDriveState);
//        telemetry.addData("POSITION - Current X",  "%.2f", positionTracker.getXPositionCM());
//        telemetry.addData("POSITION - Current Y",  "%.2f", positionTracker.getYPositionCM());
//        telemetry.addData("POSITION - Current heading", positionTracker.getHeadingDegrees());
//
//        telemetry.addData("TARGET POS - X", targetXCM);
//        telemetry.addData("TARGET POS - Y", targetYCM);
//        telemetry.addData("DELTA POS - X", deltaX);
//        telemetry.addData("DELTA POS - Y", deltaY);
//
//        telemetry.addData("TARGET HEADING", targetHeadingDegrees);
//        telemetry.addData("TARGET HEADING DELTA", Math.toDegrees(deltaHeading));
//        telemetry.addData("DISTANCE TO TARGET",distanceToTarget);
//
//        // Step 1: Move to the target position
//        if (Math.abs(distanceToTarget) > POSITION_TOLERANCE_CM && tmpDriveState == driveToPositionState.DRIVE) {
//            // Calculate the angle to the target relative to the robot's position
////            double angleToTarget = Math.atan2(deltaY, deltaX);
////            double relativeAngleToTarget = angleToTarget - currentHeading;
////
////            // Normalize the relative angle to the range [-π, π]
////            if (relativeAngleToTarget > Math.PI) relativeAngleToTarget -= 2 * Math.PI;
////            if (relativeAngleToTarget < -Math.PI) relativeAngleToTarget += 2 * Math.PI;
////
////            // PID Control
////            double derivativeX = deltaX - Drive_previousErrorX;
////            double derivativeY = deltaY - Drive_previousErrorY;
////            double forwardPower = (Drive_Kp * deltaX) + (Drive_Kd * derivativeX);
////            double strafePower = (Drive_Kp * deltaY) + (Drive_Kd * derivativeY);
////
////            // Store previous errors
////            Drive_previousErrorX = deltaX;
////            Drive_previousErrorY = deltaY;
//            // Calculate the angle to the target relative to the robot's position
//            double angleToTarget = Math.atan2(deltaY, deltaX);
//            double relativeAngleToTarget = angleToTarget - currentHeading;
//
//            // Normalize the relative angle
//            if (relativeAngleToTarget > Math.PI) relativeAngleToTarget -= 2 * Math.PI;
//            if (relativeAngleToTarget < -Math.PI) relativeAngleToTarget += 2 * Math.PI;
//
//            // Transform deltaX and deltaY into robot's coordinate frame
//            double robotRelativeX = distanceToTarget * Math.cos(relativeAngleToTarget);
//            double robotRelativeY = distanceToTarget * Math.sin(relativeAngleToTarget);
//
//            // PID Control using robot-relative coordinates
//            double derivativeX = robotRelativeX - Drive_previousErrorX;
//            double derivativeY = robotRelativeY - Drive_previousErrorY;
//            double forwardPower = (Drive_Kp * robotRelativeX) + (Drive_Kd * derivativeX);
//            double strafePower = (Drive_Kp * robotRelativeY) + (Drive_Kd * derivativeY);
//
//            // Store previous errors in robot-relative coordinates
//            Drive_previousErrorX = robotRelativeX;
//            Drive_previousErrorY = robotRelativeY;
//
//            // Normalize power values to prevent exceeding max power
//            double maxPower = Math.max(Math.abs(forwardPower), Math.abs(strafePower));
//            if (maxPower > 1.0) {
//                forwardPower /= maxPower;
//                strafePower /= maxPower;
//            }
//
//            // Apply smooth deceleration when close to target (within DRIVE_SLOW_THRESHOLD)
//            if (distanceToTarget < DRIVE_SLOW_THRESHOLD) {
//                // Apply a smooth decrease in power based on how close we are to the target
//                double slowDownFactor = distanceToTarget / DRIVE_SLOW_THRESHOLD; // This will reduce power smoothly as we approach
//                forwardPower *= slowDownFactor;
//                strafePower *= slowDownFactor;
//
//                // Ensure the power stays above DRIVE_MIN_POWER in the active direction (either forward or strafe)
//                if (Math.abs(deltaX) > Math.abs(deltaY)) {
//                    // Moving in the X direction, ensure forwardPower is at least DRIVE_MIN_POWER
//                    forwardPower = Math.max(DRIVE_MIN_POWER, Math.abs(forwardPower)) * Math.signum(forwardPower);  // Ensure forwardPower is at least DRIVE_MIN_POWER and keeps its sign
//                    // Allow strafePower to go to zero
//                    strafePower = 0;
//                } else {
//                    // Moving in the Y direction, ensure strafePower is at least DRIVE_MIN_POWER
//                    strafePower = Math.max(DRIVE_MIN_POWER, Math.abs(strafePower)) * Math.signum(strafePower);  // Ensure strafePower is at least DRIVE_MIN_POWER and keeps its sign
//                    // Allow forwardPower to go to zero
//                    forwardPower = 0;
//                }
//            }
//
//            // Send adjusted power to the drive system
//            driveWheels(forwardPower, 0, -strafePower, false, driveSlow);
//
//        } else {
//            if (tmpDriveState == driveToPositionState.DRIVE) {
//
//                tmpDriveState = driveToPositionState.TURN;
//                //tmpDriveState = driveToPositionState.COMPLETE;
//                //tmpDriveState = driveToPositionState.ADJUST;
//                driveWheels(0, 0, 0, false, driveSlow); // Stop movement
//
//            }
//        }
//
//        // Step 2: Adjust heading if needed
//        if (Math.abs(deltaHeading) > Math.toRadians(HEADING_TOLERANCE_DEGREES) && tmpDriveState == driveToPositionState.TURN) {
//            // PID Controller for turning
//            double turnDerivative = deltaHeading - Turn_PrevDelta;
//
//            // PID formula (without Ki)
//            double turnPower = (TURN_KP * deltaHeading) + (TURN_KD * turnDerivative);
//
//            double slowDownFactor = Math.min(1.0, Math.abs(deltaHeading) / TURN_SLOWDOWN_THRESHOLD);
//            turnPower *= slowDownFactor;
//
//            // Normalize turnPower based on the max power
//            if (Math.abs(turnPower) > TURN_MAX_POWER) {
//                turnPower = TURN_MAX_POWER * Math.signum(turnPower);
//            }
//
//            // Apply minimum power to overcome friction
//            if (Math.abs(turnPower) < TURN_MIN_POWER) {
//                turnPower = TURN_MIN_POWER * Math.signum(turnPower);
//            }
//
//            telemetry.addData("turnPower", (turnPower));
//
//            // Apply turn power
//            driveWheels(0, -turnPower, 0, false, false);
//
//            // Store the current error for the next cycle
//            Turn_PrevDelta = deltaHeading;
//        } else {
//            if (tmpDriveState == driveToPositionState.TURN) {
//                driveWheels(0, 0, 0, false, driveSlow);
//                tmpDriveState = driveToPositionState.ADJUST;
//            }
//        }
//
//
//
//        // Step 3: Adjust position if needed (after all other movements)
//        //if (Math.abs(distanceToTarget) > POSITION_FINE_TOLERANCE_CM && tmpDriveState == driveToPositionState.ADJUST) {
//        telemetry.addData("DELTA X > TOLERANCE",(Math.abs(deltaX) > POSITION_ADJUST_TOLERANCE_CM));
//        telemetry.addData("DELTA Y > TOLERANCE",(Math.abs(deltaY) > POSITION_ADJUST_TOLERANCE_CM));
//        if ((Math.abs(deltaX) > POSITION_ADJUST_TOLERANCE_CM || Math.abs(deltaY) > POSITION_ADJUST_TOLERANCE_CM) && tmpDriveState == driveToPositionState.ADJUST) {
//            // Fine adjustment state - move one direction at a time
//            if (Math.abs(deltaX) > Math.abs(deltaY)) {
//                // Move only in the X direction
//                driveWheels(Math.signum(deltaX) * DRIVE_MIN_POWER, 0, 0, false, driveSlow); // Move only in the X direction
//            } else {
//                // Move only in the Y direction
//                driveWheels(0, 0, -(Math.signum(deltaY) * DRIVE_MIN_POWER), false, driveSlow); // Move only in the Y direction
//            }
//        } else {
//            if (tmpDriveState == driveToPositionState.ADJUST) {
//                tmpDriveState = driveToPositionState.COMPLETE;
//                driveWheels(0, 0, 0, false, driveSlow); // Stop movement
//            }
//        }
//
//        return tmpDriveState == driveToPositionState.COMPLETE;
//    }

    public void resetDrivePosition() {
        tmpDriveState = driveToPositionState.IDLE;
    }

//    private double calculateDrivePower(double distance) {
//        // If within crawlThreshold, apply minimum power to keep moving
//        if (Math.abs(distance) <= DRIVE_CRAWL_THRESHOLD) {
//            return Math.signum(distance) * DRIVE_MIN_POWER;
//        }
//
//        // Scale power based on slowDownThreshold
//        double power;
//        if (Math.abs(distance) > DRIVE_SLOW_THRESHOLD) {
//            power = DRIVE_MAX_POWER; // Full speed when far away
//        } else {
//            power = Math.max(DRIVE_MIN_POWER, (Math.abs(distance) / DRIVE_SLOW_THRESHOLD) * DRIVE_MAX_POWER);
//        }
//
//        // Ensure power direction matches movement
//        return (distance < 0) ? -power : power;
//    }
//
//    private double[] calculateDrivePower(double errorX, double errorY, double errorHeading) {
//        // Proportional term
//        double proportionalX = Drive_Kp * errorX;
//        double proportionalY = Drive_Kp * errorY;
//
//        // Reset integral term if the error is small (prevents windup)
//        if (Math.abs(errorX) < POSITION_TOLERANCE_CM) Drive_integralX = 0;
//        if (Math.abs(errorY) < POSITION_TOLERANCE_CM) Drive_integralY = 0;
//
//        // Accumulate integral term
//        Drive_integralX += errorX;
//        Drive_integralY += errorY;
//
//        // Clamp integral to prevent excessive buildup
//        Drive_integralX = Math.max(-INTEGRAL_CAP, Math.min(INTEGRAL_CAP, Drive_integralX));
//        Drive_integralY = Math.max(-INTEGRAL_CAP, Math.min(INTEGRAL_CAP, Drive_integralY));
//
//        // Compute integral term
//        double integralTermX = Drive_Ki * Drive_integralX;
//        double integralTermY = Drive_Ki * Drive_integralY;
//
//        // Compute derivative term (set Drive_Kd to 0 initially to reduce stuttering)
//        double derivativeTermX = 0; // Drive_Kd * (errorX - Drive_previousErrorX);
//        double derivativeTermY = 0; // Drive_Kd * (errorY - Drive_previousErrorY);
//
//        // Update previous errors
//        Drive_previousErrorX = errorX;
//        Drive_previousErrorY = errorY;
//
//        // PID output for X and Y
//        double pidOutputX = proportionalX + integralTermX + derivativeTermX;
//        double pidOutputY = proportionalY + integralTermY + derivativeTermY;
//
//        // Scale power smoothly instead of forcing a min power
//        double scaleFactorX = Math.min(1.0, Math.abs(errorX) / POSITION_TOLERANCE_CM);
//        double scaleFactorY = Math.min(1.0, Math.abs(errorY) / POSITION_TOLERANCE_CM);
//
//        pidOutputX *= scaleFactorX;
//        pidOutputY *= scaleFactorY;
//
//        // Apply rotation adjustments
//        double forwardPower = Math.cos(errorHeading) * pidOutputX + Math.sin(errorHeading) * pidOutputY;
//        double strafePower = -Math.sin(errorHeading) * pidOutputX + Math.cos(errorHeading) * pidOutputY;
//
//        // Clamp power after rotation adjustments
//        forwardPower = Math.max(-DRIVE_MAX_POWER, Math.min(DRIVE_MAX_POWER, forwardPower));
//        strafePower = Math.max(-DRIVE_MAX_POWER, Math.min(DRIVE_MAX_POWER, strafePower));
//
//        return new double[]{forwardPower, strafePower};
//    }


    //    private double calculateTurnPower(double deltaHeading) {
//        // If within the final tolerance range, stop turning
//        if (Math.abs(deltaHeading) <= Math.toRadians(HEADING_TOLERANCE_DEGREES)) {
//            telemetry.addData("TURN ","STOP");
//            return 0.0; // Stop turning when within tolerance range
//        }
//
//        // If the robot is within the slow down threshold, apply the slow turn power
//        if (Math.abs(deltaHeading) <= TURN_SLOWDOWN_THRESHOLD) {
//            telemetry.addData("TURN ","SLOW");
//            return TURN_MIN_POWER;// * Math.signum(deltaHeading); // Slow down the turn
//        }
//        telemetry.addData("TURN ","FULL");
//        // Apply full turn power for larger heading differences
//        return TURN_MAX_POWER;//maxTurnPower * Math.signum(deltaHeading); // Apply max turn power
//    }
//

    public void moveArm(float leftStickY) {
        if (leftStickY != 0) {
            tmpArmState = manualArmState.MOVE_ARM;
            telemetry.addData("ACTION", "gamepad2.left_stick_y");
            telemetry.addData("GamepadY", leftStickY);
            // Get the value of the left stick Y axis (range from -1.0 to 1.0)

            // Invert the stick input for natural control (up = positive value, down = negative value)
            leftStickY = -leftStickY;  // If you want the arm to move up when the stick is pulled up

            // Adjust the target position based on the stick value (scaled by max speed)
            //armTargetPosition += (leftStickY * ARM_MAX_SPEED);  // Increment or decrement target position
            armTargetPosition += (leftStickY * ARM_MANUAL_MAX_SPEED);
            // Calculate dynamicArmMinPosition based on extensionPosition (to ensure the arm does not move too low)
            int extensionPosition = extensionArmMotor.getCurrentPosition();
//            if (extensionPosition == EXTENSION_MAX_POSITION) {
//                dynamicArmMinPosition = 200;
//            } else if (extensionPosition >= 900) {
//                // Linearly interpolate between 900 and 2200 for the range [100, 200]
//                //this code may not work correctly
//                dynamicArmMinPosition = (int) (100 + (extensionPosition - 900) * (200 - 100) / (EXTENSION_MAX_POSITION - 900));
//            } else if (extensionPosition < 900 && extensionPosition > EXTENSION_MIN_POSITION) {
//                dynamicArmMinPosition = ARM_MIN_POSITION;
//            } else {
//                dynamicArmMinPosition = 0;
//            }

            // Clamp the target position to ensure the arm doesn't exceed the boundaries
            //armTargetPosition = Range.clip(armTargetPosition, dynamicArmMinPosition, ARM_MAX_POSITION);
            armTargetPosition = Range.clip(armTargetPosition, ARM_MIN_POSITION, ARM_MAX_POSITION);
            // Set the arm motor's target position
            armMotor.setTargetPosition(armTargetPosition);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            currentArmPower = calcArmPower();
            // Set motor power based on your control system logic
            armMotor.setPower(currentArmPower);
        } else {
            if (tmpArmState == manualArmState.MOVE_ARM) {
                tmpArmState = manualArmState.IDLE;
                //armMotor.setPower(calcArmPower());
                armMotor.setPower(ARM_BASE_POWER);
            }
        }
    }

    public void moveExtension(float rightStickY) {
        if (rightStickY != 0) {
            tmpExtensionState = manualExtensionState.MOVE_EXTENSION;
            telemetry.addData("ACTION", "gamepad2.right_stick_y");
            // Invert the stick input for natural control (up = positive value, down = negative value)
            rightStickY = -rightStickY;  // If you want the arm to move up when the stick is pulled up
            extensionTargetPosition += (rightStickY * EXTENSION_MANUAL_MAX_SPEED);  // Increment or decrement target position

            // Clamp the target position to ensure the arm doesn't exceed the boundaries
            extensionTargetPosition = Range.clip(extensionTargetPosition, EXTENSION_MIN_POSITION, EXTENSION_MAX_POSITION);

            // Set the arm motor's target position
            extensionArmMotor.setTargetPosition(extensionTargetPosition);
            extensionArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            currentExtensionPower = calcExtensionPower();
            extensionArmMotor.setPower(currentExtensionPower);
        } else {
            if (tmpExtensionState == manualExtensionState.MOVE_EXTENSION) {
                tmpExtensionState = manualExtensionState.IDLE;
                extensionArmMotor.setPower(0);
            }
        }
    }

    public Boolean moveExtensionEncoder(int startPosition, int targetPosition) {
        Boolean complete = false;
        int currentPos = extensionArmMotor.getCurrentPosition();
        int distanceToTarget = Math.abs(targetPosition - currentPos);
        int distanceFromStart = Math.abs(currentPos - startPosition);
        double currentSpeed = 0;
        if (currentPos >= (targetPosition - MOTOR_TOLERANCE) && currentPos <= (targetPosition + MOTOR_TOLERANCE)) {
            complete = true;
            extensionArmMotor.setPower(0);
            //extensionArmMotor.setPower();
        } else {
            // State 1: Ramp-Up (when within the first XXX ticks of movement)
            if (distanceFromStart <= EXTENSION_RAMP_TICKS) {
                currentSpeed = EXTENSION_MAX_SPEED + (EXTENSION_MAX_SPEED - EXTENSION_MIN_SPEED) * ((float) distanceFromStart / EXTENSION_RAMP_TICKS);
            }
            // State 2: Full Speed (between the first and last 50 ticks)
            else if (distanceFromStart > EXTENSION_RAMP_TICKS && distanceToTarget > EXTENSION_RAMP_TICKS) {
                currentSpeed = EXTENSION_MAX_SPEED;
            }
            // State 3: Ramp-Down (when within the last XXX ticks of movement)
            else if (distanceToTarget <= EXTENSION_RAMP_TICKS) {
                currentSpeed = EXTENSION_MIN_SPEED + (EXTENSION_MAX_SPEED - EXTENSION_MIN_SPEED) * ((float) distanceToTarget / EXTENSION_RAMP_TICKS);
            }

            // Set the target position and motor power
            extensionTargetPosition = targetPosition;
            extensionArmMotor.setTargetPosition(extensionTargetPosition);
            extensionArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            extensionArmMotor.setPower(currentSpeed);
        }
        return complete;
    }

    public int getCurrentExtensionPosition() {
        return extensionArmMotor.getCurrentPosition();
    }

    public int getCurrentArmPosition() {
        return armMotor.getCurrentPosition();
    }

    //raise or lower the robot's arm to a specific height
    public Boolean moveArmEncoder(int startPosition, int targetPosition) {
        Boolean complete = false;
        int currentPos = armMotor.getCurrentPosition();
        int distanceToTarget = Math.abs(targetPosition - currentPos);
        int distanceFromStart = Math.abs(currentPos - startPosition);
        double currentSpeed = 0;
        if (currentPos >= (targetPosition - MOTOR_TOLERANCE) && currentPos <= (targetPosition + MOTOR_TOLERANCE)) {
            complete = true;
            armMotor.setPower(ARM_BASE_POWER);
        } else {
            // State 1: Ramp-Up (when within the first XXX ticks of movement)
            if (distanceFromStart <= ARM_RAMP_TICKS) {
                currentSpeed = ARM_MIN_SPEED + (ARM_MAX_SPEED - ARM_MIN_SPEED) * ((float) distanceFromStart / ARM_RAMP_TICKS);
            }
            // State 2: Full Speed (between the first and last 50 ticks)
            else if (distanceFromStart > ARM_RAMP_TICKS && distanceToTarget > ARM_RAMP_TICKS) {
                currentSpeed = ARM_MAX_SPEED;
            }
            // State 3: Ramp-Down (when within the last XXX ticks of movement)
            else if (distanceToTarget <= ARM_RAMP_TICKS) {
                currentSpeed = ARM_MIN_SPEED + (ARM_MAX_SPEED - ARM_MIN_SPEED) * ((float) distanceToTarget / ARM_RAMP_TICKS);
            }

            // Set the target position and motor power
            armTargetPosition = targetPosition;
            armMotor.setTargetPosition(armTargetPosition);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(currentSpeed);
        }
        return complete;
    }

    public void moveIntake(boolean inward, boolean outward) {
        // --- WHEEL SERVO CONTROL ---
        if (inward || outward) {
            if (inward) {
                tmpServoState = manualServoState.INPUT;
                leftWheelServo.setPosition(SERVO_BACKWARD);  // Spin inward
                rightWheelServo.setPosition(SERVO_FORWARD); // Spin inward
            } else if (outward) {
                tmpServoState = manualServoState.OUTPUT;
                leftWheelServo.setPosition(SERVO_FORWARD);  // Spin outward
                rightWheelServo.setPosition(SERVO_BACKWARD); // Spin outward
            }
        } else {
            if (tmpServoState == manualServoState.INPUT || tmpServoState == manualServoState.OUTPUT) {
                tmpServoState = manualServoState.IDLE;
                leftWheelServo.setPosition(SERVO_STOPPED);  // Neutral
                rightWheelServo.setPosition(SERVO_STOPPED); // Neutral
            }
        }
        // --- END WHEEL SERVO CONTROL ---
    }

    public boolean moveIntakeTimed(boolean inward, boolean outward, int milliseconds, boolean stopOnCapture, LinearOpMode opMode) {
        // If this is the first call, initialize the timer and start the servos
        if (intakeTimer == null) {  // Add this as a class field: private ElapsedTime intakeTimer;
            intakeTimer = new ElapsedTime();
            if (inward || outward) {
                if (inward) {
                    tmpServoState = manualServoState.INPUT;
                    leftWheelServo.setPosition(SERVO_BACKWARD);  // Spin inward
                    rightWheelServo.setPosition(SERVO_FORWARD);  // Spin inward
                } else if (outward) {
                    tmpServoState = manualServoState.OUTPUT;
                    leftWheelServo.setPosition(SERVO_FORWARD);   // Spin outward
                    rightWheelServo.setPosition(SERVO_BACKWARD); // Spin outward
                }
                resetSampleCaptured();
            }
        }

        // Check if we should stop
        boolean shouldStop = false;

        if (intakeTimer.milliseconds() >= milliseconds) {
            shouldStop = true;
        }
        if (stopOnCapture && isSampleCaptured()) {
            shouldStop = true;
        }
        checkSampleCaptured();

        // If we should stop, clean up and return true
        if (shouldStop) {
            tmpServoState = manualServoState.IDLE;
            leftWheelServo.setPosition(SERVO_STOPPED);  // Neutral
            rightWheelServo.setPosition(SERVO_STOPPED); // Neutral
            intakeTimer = null;  // Reset timer for next use
            return true;
        }

        return false;  // Not done yet
    }

    //    public boolean moveIntakeTimed(boolean inward,boolean outward, int milliseconds, boolean stopOnCapture, LinearOpMode opMode) {
//        // Create an instance of ElapsedTime to track the duration
//        ElapsedTime timer = new ElapsedTime();
//        boolean complete = false;
//        // --- WHEEL SERVO CONTROL ---
//        if (inward || outward) {
//            if (inward) {
//                tmpServoState = manualServoState.INPUT;
//                leftWheelServo.setPosition(SERVO_BACKWARD);  // Spin inward
//                rightWheelServo.setPosition(SERVO_FORWARD);  // Spin inward
//            } else if (outward) {
//                tmpServoState = manualServoState.OUTPUT;
//                leftWheelServo.setPosition(SERVO_FORWARD);   // Spin outward
//                rightWheelServo.setPosition(SERVO_BACKWARD); // Spin outward
//            }
//
//            // Reset and start the timer
//            timer.reset();
//            resetSampleCaptured();
//            // Continue running until the specified duration has elapsed
//            boolean keepRunning = true;
//            //while (opMode.opModeIsActive() && (timer.milliseconds() < milliseconds || (stopOnCapture && !isSampleCaptured()))) {
//            while (opMode.opModeIsActive() && keepRunning) {
//
//                if (timer.milliseconds() >= milliseconds) {
//                    keepRunning = false;
//                }
//                if (stopOnCapture && isSampleCaptured()) {
//                    keepRunning = false;
//                }
//                checkSampleCaptured();
//            }
//            if (!keepRunning) {
//                // Time has elapsed; stop the servos
//                tmpServoState = manualServoState.IDLE;
//                leftWheelServo.setPosition(SERVO_STOPPED);  // Neutral
//                rightWheelServo.setPosition(SERVO_STOPPED); // Neutral
//                complete = true;
//            }
//
//        }
//        return complete;
//    }
//
    public void moveWrist(boolean up, boolean down) {
        if (up || down) {
            if (up) {
                tempServoState = wristServoState.INPUT;
                Wrist.setPosition(0.7);
            }
            if (down) {
                tempServoState = wristServoState.OUTPUT;
                Wrist.setPosition(0.3);
            }
        } else {
            if (tempServoState == wristServoState.INPUT || tempServoState == wristServoState.OUTPUT) {
                tempServoState = wristServoState.IDLE;
                Wrist.setPosition(SERVO_STOPPED);  // Neutral

            }
        }
    }

    public double calcArmPower() {
        // Adjust arm motor power based on vertical arm position
        double horizontalArmPower = ARM_BASE_POWER;
        double verticalFactor = (double) extensionTargetPosition / EXTENSION_MAX_POSITION;
        horizontalArmPower += verticalFactor * ARM_EXTRA_FORCE; // Add extra power when fully raised
        horizontalArmPower = Range.clip(horizontalArmPower, 0.2, 1.0);
        return horizontalArmPower;
    }

    public double calcExtensionPower() {
        // Get the current position of the extension arm
        int currentPosition = extensionArmMotor.getCurrentPosition();

        // Adjust the motor power based on proximity to target
        double extensionProximityFactor = 1.0; // Full power by default

        // Scale power when near target position (within 50 ticks)
        int proximityRange = 50;
        if (currentPosition < extensionTargetPosition) {
            // Arm is moving towards the max position
            if (extensionTargetPosition - currentPosition < proximityRange) {
                extensionProximityFactor = (extensionTargetPosition - currentPosition) / (double) proximityRange;
            }
        } else if (currentPosition > extensionTargetPosition) {
            // Arm is moving towards the min position
            if (currentPosition - extensionTargetPosition < proximityRange) {
                extensionProximityFactor = (currentPosition - extensionTargetPosition) / (double) proximityRange;
            }
        }

        // Calculate the base power based on current position relative to the target
        double distanceToTarget = Math.abs(extensionTargetPosition - currentPosition);
        double extensionPower = EXTENSION_BASE_POWER + ((double) distanceToTarget / EXTENSION_MAX_POSITION) * EXTENSION_EXTRA_FORCE;

        // Apply proximity factor to the power (if close to target, reduce power)
        extensionPower = extensionPower * extensionProximityFactor;

        // Clip the power to a safe range (minimum 0.2, maximum 1.0)
        extensionPower = Range.clip(extensionPower, 0.2, 1.0);

        // Return the calculated power
        return extensionPower;
    }

    public void setDefaultPower() {
        armMotor.setTargetPosition(armTargetPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //currentArmPower = calcArmPower();
        armMotor.setPower(ARM_BASE_POWER);

        extensionArmMotor.setTargetPosition(extensionTargetPosition);
        extensionArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        currentExtensionPower = 0;
        extensionArmMotor.setPower(currentExtensionPower);

        moveIntake(false, false);
    }


    //EMERGENCY RESET BUTTON (BOTTOM)
    public void emergencyReset() {
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armTargetPosition = 0;
        armMotor.setTargetPosition(armTargetPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0);

        // Stop and reset encoder
        extensionArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        extensionArmMotor.setPower(0);

        extensionArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extensionTargetPosition = 0;
        extensionArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionArmMotor.setTargetPosition(0);
        extensionArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }

    public void specimenHook() {
        telemetry.addData("CURRENT ACTION STATE", specimenHookState);
        //SPECIMEN HOOK
        if (specimenHookState == HookState.IDLE) {
            specimenHookState = HookState.POSITION_ROBOT; // Start first step

        }
        tmpExtensionPositionHolder = extensionArmMotor.getCurrentPosition();
        tmpArmPositionHolder = armMotor.getCurrentPosition();

        // Execute multi-step process based on current state
        switch (specimenHookState) {
            case POSITION_ROBOT:
                if (driveToPosition(HOOK_POS_X, HOOK_POS_Y, HOOK_DEGREES, false) && moveArmEncoder(tmpArmPositionHolder, DRIVE_ARM_POSITION)) {
                    resetDrivePosition();
                    specimenHookState = HookState.PLACE_ARM; // Transition to next step
                }
                break;
//              Gamepad 1 y hook (without 2nd gamempad!!!!!!!!
//            case PLACE_ARM:
//                if (moveArmEncoder(tmpArmPositionHolder,HOOK_ARM_HEIGHT) && moveExtensionEncoder(tmpExtensionPositionHolder,HOOK_EXTENSION_POSITION)) {
//                    specimenHookState = HookState.DROP_ARM; // Transition to next step
//                }
//                break;
//            case DROP_ARM:
//                if (moveArmEncoder(tmpArmPositionHolder,HOOK_ARM_HEIGHT_2)) {
//                    specimenHookState = HookState.MOVE_OUT; // Transition to next step
//                    tmpActionStartTime = System.currentTimeMillis();
//                    tmpExtensionPositionHolder = extensionArmMotor.getCurrentPosition();
//                }
//                break;
//            case MOVE_OUT:
//                long elapsedTime = System.currentTimeMillis() - tmpActionStartTime;
//                moveIntake(false, true);
//                if (elapsedTime > 100) {
//                    specimenHookState = HookState.HOOK_ARM; // Transition to next step
//                }
//                break;
//            case HOOK_ARM:
//                moveIntake(false,false);
//                if (moveExtensionEncoder(tmpExtensionPositionHolder,0)) {
//                    specimenHookState = HookState.COMPLETE; // Transition to next step
//                }
//                break;
            case COMPLETE:
                setDefaultPower();
                break;
        }

    }

    public void sampleRelease() {
        telemetry.addData("CURRENT ACTION STATE", sampleReleaseState);
        //SAMPLE RELEASE TO FIRST BUCKET
        if (sampleReleaseState == releaseSampleFirstBucketState.IDLE) {
            sampleReleaseState = releaseSampleFirstBucketState.POSITION_ROBOT; // Start first step

        }
        tmpExtensionPositionHolder = extensionArmMotor.getCurrentPosition();
        tmpArmPositionHolder = armMotor.getCurrentPosition();

        // Execute multi-step process based on current state
        switch (sampleReleaseState) {
            case POSITION_ROBOT:
                if (driveToPosition(RELEASE_SAMPLE_POS_X, RELEASE_SAMPLE_POS_Y, RELEASE_SAMPLE_DEGREES, false) && moveArmEncoder(tmpArmPositionHolder, DRIVE_ARM_POSITION)) {
                    resetDrivePosition();
                    sampleReleaseState = releaseSampleFirstBucketState.MOVE_ARM; // Transition to next step
                }
                break;
            case MOVE_ARM:
                if (moveArmEncoder(tmpExtensionPositionHolder, RELEASE_SAMPLE_ARM_HEIGHT)) {
                    sampleReleaseState = releaseSampleFirstBucketState.EXTEND; // Transition to next step
                    tmpExtensionPositionHolder = extensionArmMotor.getCurrentPosition();
                    tmpActionStartTime = System.currentTimeMillis();
                }
                break;
            case EXTEND:
                if (moveExtensionEncoder(tmpArmPositionHolder, RELEASE_SAMPLE_EXTENSION_POSITION)) {
                    sampleReleaseState = releaseSampleFirstBucketState.DROP_ARM; // Transition to next step
                    tmpExtensionPositionHolder = extensionArmMotor.getCurrentPosition();
                    tmpActionStartTime = System.currentTimeMillis();
                }
                break;
            case DROP_ARM:
                if (moveArmEncoder(tmpExtensionPositionHolder, RELEASE_SAMPLE_ARM_HEIGHT_2)) {
                    sampleReleaseState = releaseSampleFirstBucketState.OUTTAKE; // Transition to next step
                    tmpExtensionPositionHolder = extensionArmMotor.getCurrentPosition();
                    tmpActionStartTime = System.currentTimeMillis();
                }
                break;
            case OUTTAKE:
                long outtakeTime = System.currentTimeMillis() - tmpActionStartTime;
                telemetry.addData("OUTTAKE", "Elapsed Time: " + outtakeTime + " ms");
                // Move the intake motor
                moveIntake(false, true);
                if (outtakeTime > 1500) {
                    moveIntake(false, false);
                    if (moveExtensionEncoder(tmpExtensionPositionHolder, EXTENSION_MIN_POSITION)) {
                        sampleReleaseState = releaseSampleFirstBucketState.COMPLETE; // Transition to complete step
                    }
                }
                break;
            case COMPLETE:
                setDefaultPower();
                break;
        }
    }

    public void samplePickupGround() {
        telemetry.addData("CURRENT ACTION STATE", samplePickupState);
        //SAMPLE PICKUP FROM GROUND
        if (samplePickupState == pickupSampleGroundState.IDLE.IDLE) {
            samplePickupState = pickupSampleGroundState.POSITION_ROBOT; // Start first step

        }
        tmpExtensionPositionHolder = extensionArmMotor.getCurrentPosition();
        tmpArmPositionHolder = armMotor.getCurrentPosition();

        // Execute multi-step process based on current state
        switch (samplePickupState) {
            case POSITION_ROBOT:
                if (driveToPosition(PICKUP_SAMPLE_POS_X, PICKUP_SAMPLE_POS_Y, PICKUP_SAMPLE_DEGREES, false) && moveArmEncoder(tmpArmPositionHolder, DRIVE_ARM_POSITION)) {
                    samplePickupState = pickupSampleGroundState.MOVE_ARM; // Transition to next step
                    resetDrivePosition();
                }
                break;
            case MOVE_ARM:
                if (moveArmEncoder(tmpArmPositionHolder, PICKUP_SAMPLE_ARM_HEIGHT) && moveExtensionEncoder(tmpExtensionPositionHolder, PICKUP_SAMPLE_EXTENSION_POSITION)) { // && moveWrist) {
                    tmpActionStartTime = System.currentTimeMillis();
                    samplePickupState = pickupSampleGroundState.INTAKE; // Transition to next step
                    resetSampleCaptured();
                }
                break;
            case INTAKE:
                long intakeTime = System.currentTimeMillis() - tmpActionStartTime;
                boolean atPos = false;
                telemetry.addData("INTAKE", "Elapsed Time: " + intakeTime + " ms");
                // Move the intake motor

                moveIntake(true, false);
                checkSampleCaptured();
                driveToPosition(PICKUP_SAMPLE_POS_INTAKE_X, PICKUP_SAMPLE_POS_Y, PICKUP_SAMPLE_DEGREES, true);

                if ((intakeTime > 3500 || isSampleCaptured())) {
                    resetDrivePosition();
                    moveIntake(false, false);
                    if (isSampleCaptured()) {
                        if (moveArmEncoder(tmpArmPositionHolder, DRIVE_ARM_POSITION) && moveExtensionEncoder(tmpExtensionPositionHolder, EXTENSION_MIN_POSITION)) {
                            samplePickupState = pickupSampleGroundState.COMPLETE; // Transition to complete step
                        }
                    } else {
                        samplePickupState = pickupSampleGroundState.NOPICKUP; // Transition to the nopickup state
                    }
                }
                break;
            case NOPICKUP:
                if (driveToPosition(PICKUP_SAMPLE_POS_NOPICKUP_X, PICKUP_SAMPLE_POS_Y, PICKUP_SAMPLE_DEGREES, false)) {
                    resetDrivePosition();
                    long noPickupTime = System.currentTimeMillis() - tmpActionStartTime;
                    if (noPickupTime > 2000) {
                        samplePickupState = pickupSampleGroundState.POSITION_ROBOT; // Transition to next step
                    }
                }
                break;
            case COMPLETE:
                setDefaultPower();
                break;
        }
    }


    //:) PART THAT DOES THE DRIVING case step later with if statemtns for more control step 1 align the rotation in radians with center of robot and next step
    public void LimeLightDrive() {

    switch (LimeStep) {
        case 0:
            if (LimeLightDetect()) {

                loadRobotPosition();
                if (LimeX > 0.5) {
                    driveToPosition(positionTracker.currentPositionXCM, positionTracker.currentPositionYCM, positionTracker.getHeading() + Math.toRadians(5), false);
                } else if (LimeX < -0.5) {
                    driveToPosition(positionTracker.currentPositionXCM, positionTracker.getYPositionCM(), positionTracker.getHeading() + Math.toRadians(5), false);
                }else {
                    LimeStep = 1;
                }

        break;
            }
        case 1:
            moveArmEncoder(getCurrentArmPosition(),PICKUP_SAMPLE_ARM_HEIGHT);
            moveExtensionEncoder(getCurrentExtensionPosition(), PICKUP_SAMPLE_EXTENSION_POSITION);
            LimeStep = 2;
        break;
        case 2:
                driveWheels(1, 0, 0, true, true);
                moveIntakeTimed(true,false,2000,true, linearOpMode);
                if (isSampleCaptured() == true);{
                    LimeStep = 3;
                }
        break;
        case 3:
            moveExtensionEncoder(getCurrentExtensionPosition(), EXTENSION_MIN_POSITION);
        break;
      }
    }

    public void LimePipeCycle() {
        if (CURRENT_PIPELINE == 1) {
            limelight.pipelineSwitch(0);
            CURRENT_PIPELINE = 0;


        } else {
            limelight.pipelineSwitch(1);
            CURRENT_PIPELINE = 1;

        }
    }

    //I put in last seen time for limelight to reduce error in case of sudden interuption from pipeline config so it will be more consistent also this detects a specimen and plots it as well as how much area it takes up
    public boolean LimeLightDetect() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            LimeX = result.getTx();
            LimeY = result.getTy();
            LimeA = result.getTa();
            gamepad1.rumble(500, 500, 100);
            lastSeenTimeLime = 0;

            return true;
        } else {
            lastSeenTimeLime++;
            if (lastSeenTimeLime > 50) {
                LimeX = 0;
                LimeY = 0;
                LimeA = 0;
                return false;
            }else{
                return true;

            }
        }

    }


    public void samplePickupGroundArmOnly() {
        telemetry.addData("CURRENT ACTION STATE",samplePickupArmOnlyState);
        //SAMPLE PICKUP FROM GROUND
        if (samplePickupArmOnlyState == pickupSampleGroundArmOnlyState.IDLE) {
            samplePickupArmOnlyState = pickupSampleGroundArmOnlyState.MOVE_ARM; // Start first step

        }
        tmpExtensionPositionHolder = extensionArmMotor.getCurrentPosition();
        tmpArmPositionHolder = armMotor.getCurrentPosition();

        // Execute multi-step process based on current state
        switch (samplePickupArmOnlyState) {
            case MOVE_ARM:
                if (moveArmEncoder(tmpArmPositionHolder, PICKUP_SAMPLE_ARM_HEIGHT) && moveExtensionEncoder(tmpExtensionPositionHolder, PICKUP_SAMPLE_EXTENSION_POSITION)) { // && moveWrist) {
                    tmpActionStartTime = System.currentTimeMillis();
                    samplePickupArmOnlyState = pickupSampleGroundArmOnlyState.INTAKE; // Transition to next step
                    resetSampleCaptured();
                }
                break;
            case INTAKE:
                long intakeTime = System.currentTimeMillis() - tmpActionStartTime;
                boolean atPos = false;
                telemetry.addData("INTAKE", "Elapsed Time: " + intakeTime + " ms");
                // Move the intake motor

                moveIntake(true, false);
                checkSampleCaptured();
                //driveToPosition(PICKUP_SAMPLE_POS_INTAKE_X,PICKUP_SAMPLE_POS_Y,PICKUP_SAMPLE_DEGREES,true);

                if ((intakeTime > 10000 || isSampleCaptured())) {
                    resetDrivePosition();
                    moveIntake(false, false);
                    if (isSampleCaptured()) {
                        if (moveArmEncoder(tmpArmPositionHolder, DRIVE_ARM_POSITION) && moveExtensionEncoder(tmpExtensionPositionHolder, EXTENSION_MIN_POSITION)) {
                            samplePickupArmOnlyState = samplePickupArmOnlyState.COMPLETE; // Transition to complete step
                        }
//                    } else {
//                        samplePickupArmOnlyState = samplePickupArmOnlyState.NOPICKUP; // Transition to the nopickup state
                    }
                }
                break;

            case COMPLETE:
                setDefaultPower();
                break;
        }
    }


    public void specimenHookArmOnly() {
        telemetry.addData("CURRENT ACTION STATE",specHookArmOnlyState);
        //SPECIMEN HOOK
        if (specHookArmOnlyState == specimenHookArmOnlyState.IDLE) {
            specHookArmOnlyState = specimenHookArmOnlyState.PLACE_ARM; // Start first step

        }
        tmpExtensionPositionHolder = extensionArmMotor.getCurrentPosition();
        tmpArmPositionHolder = armMotor.getCurrentPosition();

        // Execute multi-step process based on current state
        switch (specHookArmOnlyState) {

            case PLACE_ARM:
                if (moveArmEncoder(tmpArmPositionHolder,HOOK_ARM_HEIGHT) && moveExtensionEncoder(tmpExtensionPositionHolder,HOOK_EXTENSION_POSITION)) {
                    specHookArmOnlyState = specimenHookArmOnlyState.DROP_ARM; // Transition to next step
                }
                break;
            case DROP_ARM:
                if (moveArmEncoder(tmpArmPositionHolder,HOOK_ARM_HEIGHT_2)) {
                    specHookArmOnlyState = specimenHookArmOnlyState.MOVE_OUT; // Transition to next step
                    tmpActionStartTime = System.currentTimeMillis();
                    tmpExtensionPositionHolder = extensionArmMotor.getCurrentPosition();
                }
                break;
            case MOVE_OUT:
                long elapsedTime = System.currentTimeMillis() - tmpActionStartTime;
                moveIntake(false, true);
                if (elapsedTime > 100) {
                    specHookArmOnlyState = specimenHookArmOnlyState.HOOK_ARM; // Transition to next step
                }
                break;
            case HOOK_ARM:
                moveIntake(false,false);
                if (moveExtensionEncoder(tmpExtensionPositionHolder,0)) {
                    specHookArmOnlyState = specimenHookArmOnlyState.COMPLETE; // Transition to next step
                }
                break;
            case COMPLETE:
                setDefaultPower();
                break;
        }

    }


    public void sampleBucketArmOnly() {
        telemetry.addData("CURRENT ACTION STATE",sampleBucketArmOnlyState);
        //SAMPLE RELEASE TO FIRST BUCKET
        if (sampleBucketArmOnlyState == releaseSampleFirstBucketArmOnlyState.IDLE) {
            sampleBucketArmOnlyState = releaseSampleFirstBucketArmOnlyState.MOVE_ARM; // Start first step

        }
        tmpExtensionPositionHolder = extensionArmMotor.getCurrentPosition();
        tmpArmPositionHolder = armMotor.getCurrentPosition();

        // Execute multi-step process based on current state
        switch (sampleBucketArmOnlyState) {
            case MOVE_ARM:
                if (moveArmEncoder(tmpExtensionPositionHolder,RELEASE_SAMPLE_ARM_HEIGHT) ) {
                    sampleBucketArmOnlyState = releaseSampleFirstBucketArmOnlyState.EXTEND; // Transition to next step
                    tmpExtensionPositionHolder = extensionArmMotor.getCurrentPosition();
                    tmpActionStartTime = System.currentTimeMillis();
                }
                break;
            case EXTEND:
                if (moveExtensionEncoder(tmpArmPositionHolder,RELEASE_SAMPLE_EXTENSION_POSITION)) {
                    sampleBucketArmOnlyState = releaseSampleFirstBucketArmOnlyState.DROP_ARM; // Transition to next step
                    tmpExtensionPositionHolder = extensionArmMotor.getCurrentPosition();
                    tmpActionStartTime = System.currentTimeMillis();
                }
                break;
            case DROP_ARM:
                if (moveArmEncoder(tmpExtensionPositionHolder,RELEASE_SAMPLE_ARM_HEIGHT_2) ) {
                    sampleBucketArmOnlyState = releaseSampleFirstBucketArmOnlyState.OUTTAKE; // Transition to next step
                    tmpExtensionPositionHolder = extensionArmMotor.getCurrentPosition();
                    tmpActionStartTime = System.currentTimeMillis();
                }
                break;
            case OUTTAKE:
                long outtakeTime = System.currentTimeMillis() - tmpActionStartTime;
                telemetry.addData("OUTTAKE", "Elapsed Time: " + outtakeTime + " ms");
                // Move the intake motor
                moveIntake(false, true);
                if (outtakeTime > 1500) {
                    moveIntake(false, false);
                    if (moveExtensionEncoder(tmpExtensionPositionHolder, EXTENSION_MIN_POSITION)) {
                        sampleBucketArmOnlyState = releaseSampleFirstBucketArmOnlyState.COMPLETE; // Transition to complete step
                    }
                }
                break;
            case COMPLETE:
                setDefaultPower();
                break;
        }
    }

    boolean manualActionRunning = false;
    public void forceActionRunning(boolean setRunning) {
        manualActionRunning = setRunning;
    }

    public boolean isActionRunning() {
        boolean returnVal = false;
        if (samplePickupState != pickupSampleGroundState.IDLE ||
            sampleReleaseState != releaseSampleFirstBucketState.IDLE ||
            specimenHookState != HookState.IDLE || manualActionRunning) {
            returnVal = true;
        }
        else {
            tmpDriveState = driveToPositionState.IDLE;
        }
        return returnVal;
    }

    public void checkSampleCaptured() {
        if (touchsensor.isPressed()) {
            sampleCaptured = true;
        }
    }

//    public void limeLightBlue(){
//        if(LimelightBlueDetect == true){
//
//        }
//    }

    public boolean isSampleCaptured() {
        return sampleCaptured;
    }

    public void resetSampleCaptured() {
        sampleCaptured = false;
    }

    public enum HookState {
        IDLE,          // Waiting for button press
        POSITION_ROBOT,
        PLACE_ARM,         // Second movement
        MOVE_OUT,
        DROP_ARM,
        HOOK_ARM,
        COMPLETE       // Process complete
    }

    public enum specimenHookArmOnlyState {
        IDLE,          // Waiting for button press
        PLACE_ARM,         // Second movement
        MOVE_OUT,
        DROP_ARM,
        HOOK_ARM,
        COMPLETE   // Process complete
    }

    public enum releaseSampleFirstBucketArmOnlyState {
        IDLE,          // Waiting for button press
        POSITION_ROBOT,
        MOVE_ARM,
        EXTEND,
        DROP_ARM,
        OUTTAKE,
        COMPLETE       // Process complete
    }

//
//    public enum HookReleaseState {
//        IDLE,          // Waiting for button press
//        RETRACT_EXTENSION,         // First movement
//        COMPLETE       // Process complete
//    }

    public enum pickupSampleGroundState {
        IDLE,          // Waiting for button press
        POSITION_ROBOT,
        MOVE_ARM,
        INTAKE,
        NOPICKUP,
        COMPLETE
    }

    public enum pickupSampleGroundArmOnlyState{
        IDLE,          // Waiting for button press
        MOVE_ARM,
        INTAKE,
        COMPLETE
    }

    public enum releaseSampleFirstBucketState {
        IDLE,          // Waiting for button press
        POSITION_ROBOT,
        MOVE_ARM,
        EXTEND,
        DROP_ARM,
        OUTTAKE,
        COMPLETE       // Process complete
    }

    public enum manualArmState {
        IDLE,          // Waiting for button press
        MOVE_ARM
    }

    public enum manualExtensionState {
        IDLE,          // Waiting for button press
        MOVE_EXTENSION
    }

    public enum manualServoState {
        IDLE,          // Waiting for button press
        INPUT,
        OUTPUT
    }

    public enum wristServoState {
        IDLE,          // Waiting for button press
        INPUT,
        OUTPUT
    }

    public enum driveToPositionState {
        IDLE,          // Waiting for button press
        DRIVE,
        TURN,
        SETTLING,
        ADJUST,
        COMPLETE
    }

//
//    public boolean saveRobotPosition(android.content.Context appContext) {
//        SharedPreferences prefs = appContext.getSharedPreferences("RobotPrefs", appContext.MODE_PRIVATE);
//        SharedPreferences.Editor editor = prefs.edit();
//        editor.putFloat("robot_x", (float) positionTracker.getXPositionCM());
//        editor.putFloat("robot_y", (float) positionTracker.getYPositionCM());
//        editor.putFloat("robot_heading", (float) positionTracker.getHeading());
//        editor.putFloat("robot_arm", (float) getCurrentArmPosition());
//        editor.putFloat("robot_extension", (float) getCurrentExtensionPosition());
//        //editor.apply(); // Commit changes asynchronously
//
//        // Force synchronous write to ensure data is saved immediately
//        boolean result = editor.commit();
//        return result && prefs.contains("robot_x");
//    }
//
//    public void loadRobotPosition(android.content.Context appContext) {
//        SharedPreferences prefs = appContext.getSharedPreferences("RobotPrefs", appContext.MODE_PRIVATE);
//        double x = prefs.getFloat("robot_x", 0);
//        double y = prefs.getFloat("robot_y", 0);
//        double heading = prefs.getFloat("robot_heading", 0);
////        double armPos = prefs.getFloat("robot_arm", 0);
////        double extPos = prefs.getFloat("robot_extension", 0);
//
//
//        positionTracker.resetPosition(x,y,heading);
//
////        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        armMotor.setTargetPosition(armPos);
//    }



    /**
     * Saves robot position directly to a file instead of using SharedPreferences
     */
    public boolean saveRobotPosition() {
        try {
            // Get the file path in the app's internal storage
            File file = new File(hardwareMap.appContext.getFilesDir(), "robot_position.txt");

            // Create the data string with position values
            String positionData =
                    String.valueOf(positionTracker.getXPositionCM()) + "," +
                            String.valueOf(positionTracker.getYPositionCM()) + "," +
                            String.valueOf(positionTracker.getHeading()) + "," +
                            String.valueOf(getCurrentArmPosition()) + "," +
                            String.valueOf(getCurrentExtensionPosition());

            // Write the data to the file
            FileOutputStream fos = new FileOutputStream(file);
            fos.write(positionData.getBytes());
            fos.flush(); // Force the data to be written immediately
            fos.close();

            // Verify file exists and has content
            boolean success = file.exists() && file.length() > 0;

            telemetry.addData("Position Saved", success ? "Success" : "Failed");
            telemetry.addData("File Path", file.getAbsolutePath());
            telemetry.addData("Data Saved", positionData);
            telemetry.update();

            return success;
        } catch (Exception e) {
            telemetry.addData("Save Error", e.getMessage());
            telemetry.update();
            return false;
        }
    }

    /**
     * Loads robot position from a file
     */
    public boolean loadRobotPosition() {
        try {
            // Get the file path in the app's internal storage
            File file = new File(hardwareMap.appContext.getFilesDir(), "robot_position.txt");

            // Check if file exists
            if (!file.exists() || file.length() == 0) {
                telemetry.addData("Position Load", "No file found or empty file");
                telemetry.update();
                return false;
            }

            // Read the data from the file
            FileInputStream fis = new FileInputStream(file);
            byte[] buffer = new byte[1024];
            int length = fis.read(buffer);
            String positionData = new String(buffer, 0, length);
            fis.close();

            // Split the data into parts
            String[] parts = positionData.split(",");
            if (parts.length >= 3) {
                double x = Double.parseDouble(parts[0]);
                double y = Double.parseDouble(parts[1]);
                double heading = Double.parseDouble(parts[2]);

                // Optional arm and extension positions
                // double armPos = (parts.length > 3) ? Double.parseDouble(parts[3]) : 0;
                // double extPos = (parts.length > 4) ? Double.parseDouble(parts[4]) : 0;

                // Log the values
                telemetry.addData("Loaded Position", "X=%.2f, Y=%.2f, H=%.2f", x, y, heading);
                telemetry.addData("Raw Data", positionData);
                telemetry.update();

                // Set the position
                positionTracker.resetPosition(x, y, heading);

                // Optional arm position setting
                // armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                // armMotor.setTargetPosition((int)armPos);

                return true;
            } else {
                telemetry.addData("Position Load", "Invalid data format");
                telemetry.update();
                return false;
            }
        } catch (Exception e) {
            telemetry.addData("Load Error", e.getMessage());
            telemetry.update();
            return false;
        }
    }
}

