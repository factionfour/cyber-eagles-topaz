package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public abstract class AutoBase extends LinearOpMode {
    // define each motor and servo
    public DcMotor frontleftDrive = null;
    public DcMotor backleftDrive = null;
    public DcMotor backrightDrive = null;
    public DcMotor frontrightDrive = null;
    public DcMotor armMotor = null;
    public DcMotor extensionArmMotor = null;
    public Servo leftWheelServo = null;
    public Servo rightWheelServo = null;
    public IMU imu;

    // Arm motor limits and power (both power and speed for encoders)
    int armTargetPosition = 0;
    int ARM_MIN_POSITION = 100;    // Minimum encoder position (fully retracted)
    int ARM_MAX_POSITION = 440; // Maximum encoder position (fully extended)
    double ARM_BASE_POWER = 0.3;
    double ARM_EXTRA_FORCE = 0.7;//extra force if the extension is out
    int ARM_RAMP_TICKS = 50; // How far to move per loop iteration
    double ARM_MAX_SPEED = 0.7;
    double ARM_MIN_SPEED = 0.3;

    // Vertical extension limits and base power (both power and speed for encoders)
    int extensionTargetPosition = 0;
    int EXTENSION_MIN_POSITION = 0;    // Minimum height (fully lowered)
    int EXTENSION_MAX_POSITION = 2200; // Maximum height (fully raised)
    double EXTENSION_MIN_SPEED = 0.3;
    double EXTENSION_MAX_SPEED = 0.7;
    double EXTENSION_BASE_POWER = 0.4; // Base power to hold position
    double EXTENSION_EXTRA_FORCE = 0.4; // Extra power when arm is extended
    int EXTENSION_RAMP_TICKS = 50; // How far to move per loop iteration

     int MOTOR_TOLERANCE = 10; // Acceptable error in encoder ticks (used for arm & extension)

    //servo limits
    double SERVO_STOPPED = 0.5;
    double SERVO_FORWARD = 1;
    double SERVO_BACKWARD = 0;

    //pre-defined positions
    int HOOK_EXTENSION_POSITION = 1900;
    int HOOK_ARM_HEIGHT = 450;
    int HOOK_RELEASE_EXTENSION_POSITION = 1400;
    int HOOK_RELEASE_ARM_HEIGHT = 300;

    int dynamicArmMinPosition = 0;
    double currentExtensionPower = 0;
    double currentArmPower = 0;
    boolean predefinedActionRunning = false;

    //CONVERSION TO CENTIMETERS
    double FORWARD_MM_SECOND = 1040;
    double BACKWARD_MM_SECOND = 1060;
    double STRAIFE_MM_SECOND = 706;

    //STRAFE MOVEMENT CONTROLS
    int STRAIFE_RAMP_TIME = 600;
    int STRAIFE_RAMP_DOWN_TIME = 600;
    double STRAFE_MIN_SPEED = 0.4;
    double STRAFE_SPEED = 0.8;
    double STRAFE_CORRECTION_GAIN = 0.10;

    //FORWARD & BACKWARD MOVEMENT CONTROLS
    double FORWARD_MIN_SPEED = 0.1;
    double FORWARD_SPEED = .7;
    int FORWARD_RAMP_TIME = 600;
    double FORWARD_CORRECTION_GAIN = 0.01;

    //TURNING MOVEMENT CONTROLS
    double Kp = 0.01; // Proportional gain
    double Kd = 0.005; // Derivative gain
    double TURN_SPEED_MIN = 0.2;
    double TURN_SPEED_MAX = 0.8;
    double ROTATE_ERROR_DEGREES =2;

    private ElapsedTime runtime = new ElapsedTime();

    public void initializeHardware() {
        frontleftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontrightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backrightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        backleftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");

        frontleftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontrightDrive.setDirection(DcMotor.Direction.REVERSE);
        frontrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backrightDrive.setDirection(DcMotor.Direction.REVERSE);
        backrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleftDrive.setDirection(DcMotor.Direction.FORWARD);
        backleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize arm motor
        armMotor = hardwareMap.get(DcMotor.class, "arm_1");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize extension arm motor
        extensionArmMotor = hardwareMap.get(DcMotor.class, "arm_extendo");
        extensionArmMotor.setDirection(DcMotor.Direction.REVERSE);
        extensionArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extensionArmMotor.setTargetPosition(0);
        extensionArmMotor.setPower(EXTENSION_BASE_POWER);
        extensionArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Initialize wheel servos
        leftWheelServo = hardwareMap.get(Servo.class, "servo_one");
        rightWheelServo = hardwareMap.get(Servo.class, "servo_two");
        leftWheelServo.setPosition(0.5); // Neutral position
        rightWheelServo.setPosition(0.5); // Neutral position

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize((new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.LEFT))));

        telemetry.addData(">", "Charlie 2 is READY for auto mode.  Press START.");
        waitForStart();
    }

    public void setInitialPosition() {
        //armMotor.setTargetPosition(ARM_MIN_POSITION);
        //armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //armMotor.setPower(ARM_BASE_POWER);
        //armTargetPosition = ARM_MIN_POSITION;
        moveArm(ARM_MIN_POSITION,1000,200);
    }

    //drive the robot forward a specific distance
    public void driveForwardMM(int distanceMM, int sleepMS) {
        long timeToTravelMM = (long) ((distanceMM / FORWARD_MM_SECOND) * 1000); // Time in milliseconds
        driveForward(timeToTravelMM, sleepMS);
    }


    //drive the robot forward a specific time
    public void driveForward(double milliseconds, int sleepMS) {
        long startTime = System.currentTimeMillis();
        double currentPower = FORWARD_MIN_SPEED;
        double correctionPower = 0;  // Initialize correction power to 0 initially
        double initialHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);// Get the initial heading (yaw) from the IMU before the strafe

        runtime.reset();

        while (opModeIsActive() && (runtime.milliseconds() < milliseconds)) {
            double elapsedTime = runtime.milliseconds();

            if (elapsedTime < FORWARD_RAMP_TIME) {
                telemetry.addData("RAMPING", "UP");
                // Ramp up phase: gradually increase power from FORWARD_MIN_SPEED to FORWARD_SPEED
                currentPower = FORWARD_MIN_SPEED + (FORWARD_SPEED - FORWARD_MIN_SPEED) * (elapsedTime / FORWARD_RAMP_TIME);
            } else if (elapsedTime < (milliseconds - FORWARD_RAMP_TIME)) {
                telemetry.addData("RAMPING", "FULL");
                // Constant power phase: maintain power at FORWARD_SPEED
                currentPower = FORWARD_SPEED;
            } else {
                telemetry.addData("RAMPING", "DOWN");
                // Ramp down phase: gradually decrease power from current speed to 0
                double timeSinceRampDown = elapsedTime - (milliseconds - FORWARD_RAMP_TIME);
                double rampDownFactor = 1.0 - (timeSinceRampDown / FORWARD_RAMP_TIME); // Ramp from 1.0 to 0
                currentPower = currentPower * rampDownFactor;  // Scale currentPower down smoothly
            }

            // Ensure currentPower stays within the range [FORWARD_MIN_SPEED, FORWARD_SPEED]
            currentPower = Math.max(FORWARD_MIN_SPEED, Math.min(currentPower, FORWARD_SPEED));

            // Calculate correction power based on heading deviation
            double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double adjustmentError = initialHeading - currentHeading;

            // Normalize the adjustment error to range [-180, 180]
            if (adjustmentError > 180) adjustmentError -= 360;
            if (adjustmentError < -180) adjustmentError += 360;

            // Ramp down correction power similarly as the main currentPower
            double correctionRampDownFactor = currentPower / FORWARD_SPEED; // Same ramp factor as currentPower
            correctionPower = FORWARD_CORRECTION_GAIN * adjustmentError * correctionRampDownFactor;


            // Apply the power to the motors (inverting the power for right motors)
            frontrightDrive.setPower(-currentPower + correctionPower);
            frontleftDrive.setPower(-currentPower - correctionPower);
            backleftDrive.setPower(currentPower + correctionPower);
            backrightDrive.setPower(currentPower - correctionPower);

            // Telemetry for monitoring
            telemetry.addData("Wheel Power", currentPower);
            telemetry.addData("CORRECTION POWER", correctionPower);
            telemetry.addData("Elapsed Time", elapsedTime);
            telemetry.update();
        }

        // Stop the motors after the loop is done
        frontleftDrive.setPower(0);
        frontrightDrive.setPower(0);
        backleftDrive.setPower(0);
        backrightDrive.setPower(0);

        sleep(sleepMS);
    }

    /*private Boolean isDriveMotorBusy() {

    }
    //drive the robot forward a specific time
    public void driveForwardTicks(double ticks, int sleepMS) {
        long startTime = System.currentTimeMillis();
        double currentPower = FORWARD_MIN_SPEED;
        double correctionPower = 0;  // Initialize correction power to 0 initially
        double initialHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);// Get the initial heading (yaw) from the IMU before the strafe

        runtime.reset();

        while (opModeIsActive() && isDriveMotorBusy()) {
            double elapsedTime = runtime.milliseconds();

            if (elapsedTime < FORWARD_RAMP_TIME) {
                telemetry.addData("RAMPING", "UP");
                // Ramp up phase: gradually increase power from FORWARD_MIN_SPEED to FORWARD_SPEED
                currentPower = FORWARD_MIN_SPEED + (FORWARD_SPEED - FORWARD_MIN_SPEED) * (elapsedTime / FORWARD_RAMP_TIME);
            } else if (elapsedTime < (milliseconds - FORWARD_RAMP_TIME)) {
                telemetry.addData("RAMPING", "FULL");
                // Constant power phase: maintain power at FORWARD_SPEED
                currentPower = FORWARD_SPEED;
            } else {
                telemetry.addData("RAMPING", "DOWN");
                // Ramp down phase: gradually decrease power from current speed to 0
                double timeSinceRampDown = elapsedTime - (milliseconds - FORWARD_RAMP_TIME);
                double rampDownFactor = 1.0 - (timeSinceRampDown / FORWARD_RAMP_TIME); // Ramp from 1.0 to 0
                currentPower = currentPower * rampDownFactor;  // Scale currentPower down smoothly
            }

            // Ensure currentPower stays within the range [FORWARD_MIN_SPEED, FORWARD_SPEED]
            currentPower = Math.max(FORWARD_MIN_SPEED, Math.min(currentPower, FORWARD_SPEED));

            // Calculate correction power based on heading deviation
            double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double adjustmentError = initialHeading - currentHeading;

            // Normalize the adjustment error to range [-180, 180]
            if (adjustmentError > 180) adjustmentError -= 360;
            if (adjustmentError < -180) adjustmentError += 360;

            // Ramp down correction power similarly as the main currentPower
            double correctionRampDownFactor = currentPower / FORWARD_SPEED; // Same ramp factor as currentPower
            correctionPower = FORWARD_CORRECTION_GAIN * adjustmentError * correctionRampDownFactor;


            // Apply the power to the motors (inverting the power for right motors)
            frontrightDrive.setPower(-currentPower + correctionPower);
            frontleftDrive.setPower(-currentPower - correctionPower);
            backleftDrive.setPower(currentPower + correctionPower);
            backrightDrive.setPower(currentPower - correctionPower);

            // Telemetry for monitoring
            telemetry.addData("Wheel Power", currentPower);
            telemetry.addData("CORRECTION POWER", correctionPower);
            telemetry.addData("Elapsed Time", elapsedTime);
            telemetry.update();
        }

        // Stop the motors after the loop is done
        frontleftDrive.setPower(0);
        frontrightDrive.setPower(0);
        backleftDrive.setPower(0);
        backrightDrive.setPower(0);

        sleep(sleepMS);
    }*/

    //drive robot backward a specific distance
    public void driveBackwardMM(int distanceMM, int sleepMS) {
        long timeToTravelMM = (long) ((distanceMM / FORWARD_MM_SECOND) * 1000); // Time in milliseconds
        driveBackward(timeToTravelMM, sleepMS);
    }

    //drive robot backward a specific time
    public void driveBackward(double milliseconds, int sleepMS) {
        long startTime = System.currentTimeMillis();
        double currentPower = FORWARD_MIN_SPEED;
        double correctionPower = 0;  // Initialize correction power to 0 initially
        double initialHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES); // Get the initial heading (yaw) from the IMU before the strafe

        runtime.reset();

        while (opModeIsActive() && (runtime.milliseconds() < milliseconds)) {
            double elapsedTime = runtime.milliseconds();

            if (elapsedTime < FORWARD_RAMP_TIME) {
                telemetry.addData("RAMPING", "UP");
                // Ramp up phase: gradually increase power from FORWARD_MIN_SPEED to FORWARD_SPEED
                currentPower = FORWARD_MIN_SPEED + (FORWARD_SPEED - FORWARD_MIN_SPEED) * (elapsedTime / FORWARD_RAMP_TIME);
            } else if (elapsedTime < (milliseconds - FORWARD_RAMP_TIME)) {
                telemetry.addData("RAMPING", "FULL");
                // Constant power phase: maintain power at FORWARD_SPEED
                currentPower = FORWARD_SPEED;
            } else {
                telemetry.addData("RAMPING", "DOWN");
                // Ramp down phase: gradually decrease power from current speed to 0
                double timeSinceRampDown = elapsedTime - (milliseconds - FORWARD_RAMP_TIME);
                double rampDownFactor = 1.0 - (timeSinceRampDown / FORWARD_RAMP_TIME); // Ramp from 1.0 to 0
                currentPower = currentPower * rampDownFactor;  // Scale currentPower down smoothly
            }

            // Ensure currentPower stays within the range [FORWARD_MIN_SPEED, FORWARD_SPEED]
            currentPower = Math.max(FORWARD_MIN_SPEED, Math.min(currentPower, FORWARD_SPEED));

            // Calculate correction power based on heading deviation
            double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double adjustmentError = initialHeading - currentHeading;

            // Normalize the adjustment error to range [-180, 180]
            if (adjustmentError > 180) adjustmentError -= 360;
            if (adjustmentError < -180) adjustmentError += 360;

            // Ramp down correction power similarly as the main currentPower
            double correctionRampDownFactor = currentPower / FORWARD_SPEED; // Same ramp factor as currentPower
            correctionPower = FORWARD_CORRECTION_GAIN * adjustmentError * correctionRampDownFactor;

            // Apply the power to the motors for backward movement
            frontrightDrive.setPower(currentPower - correctionPower);
            frontleftDrive.setPower(currentPower + correctionPower);
            backleftDrive.setPower(-currentPower - correctionPower);
            backrightDrive.setPower(-currentPower + correctionPower);

            // Telemetry for monitoring
            telemetry.addData("Wheel Power", currentPower);
            telemetry.addData("CORRECTION POWER", correctionPower);
            telemetry.addData("Elapsed Time", elapsedTime);
            telemetry.update();
        }

        // Stop the motors after the loop is done
        frontleftDrive.setPower(0);
        frontrightDrive.setPower(0);
        backleftDrive.setPower(0);
        backrightDrive.setPower(0);

        // Sleep for the specified time
        sleep(sleepMS);
    }

    //strafe robot left a specific distance
    public void strafeLeftMM(int distanceMM, int sleepMS) {
        //long timeToTravelMM = (long) ((distanceMM / STRAIFE_MM_SECOND) * 1000); // Time in milliseconds

        // Calculate ramp-up/down distance
        double rampDistanceMM = ((FORWARD_MIN_SPEED + FORWARD_SPEED) / 2) * STRAIFE_RAMP_TIME / 1000.0 * STRAIFE_MM_SECOND;
        // Ensure ramp distances don't exceed total distance
        double constantSpeedDistanceMM = Math.max(0, distanceMM - 2 * rampDistanceMM);
        // Calculate durations for each phase
        double rampTimeMS = STRAIFE_RAMP_TIME; // Ramp-up and ramp-down times are fixed
        double constantSpeedTimeMS = (constantSpeedDistanceMM / STRAIFE_MM_SECOND) * 1000;
        // Total time to travel
        long totalTimeMS = (long) (2 * rampTimeMS + constantSpeedTimeMS);
        strafeLeft(totalTimeMS, sleepMS);
    }

    //strafe robot a specific time
    public void strafeLeft(double milliseconds, int sleepMS) {
        long startTime = System.currentTimeMillis();
        double currentPower = STRAFE_MIN_SPEED;
        double correctionPower = 0;  // Initialize correction power to 0 initially
        runtime.reset();

        // Get the initial heading (yaw) from the IMU before the strafe
        double initialHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        while (opModeIsActive() && (runtime.milliseconds() < milliseconds)) {
            double elapsedTime = runtime.milliseconds();

            // Calculate the percentage of time elapsed (this can be used for ramping)
            double rampFactor = 1.0;

            // Ramp up phase (first part)
            if (elapsedTime < STRAIFE_RAMP_TIME) {
                telemetry.addData("RAMPING", "UP");
                // Ramp up phase: gradually increase power from STRAFE_MIN_SPEED to STRAFE_SPEED
                currentPower = STRAFE_MIN_SPEED + (STRAFE_SPEED - STRAFE_MIN_SPEED) * (elapsedTime / STRAIFE_RAMP_TIME);
            } else if (elapsedTime < (milliseconds - STRAIFE_RAMP_DOWN_TIME)) {
                telemetry.addData("RAMPING", "FULL");
                // Constant power phase: maintain power at FORWARD_SPEED
                currentPower = STRAFE_SPEED;
            } else {
                telemetry.addData("RAMPING", "DOWN");
                // Ramp down phase: gradually decrease power from current speed to 0
                double timeSinceRampDown = elapsedTime - (milliseconds - STRAIFE_RAMP_DOWN_TIME);
                double rampDownFactor = 1.0 - (timeSinceRampDown / STRAIFE_RAMP_DOWN_TIME); // Ramp from 1.0 to 0
                currentPower = currentPower * rampDownFactor;  // Scale currentPower down smoothly
            }

            // Ensure currentPower stays within the range [0, FORWARD_SPEED]
            currentPower = Math.max(0, Math.min(currentPower, STRAFE_SPEED));

            // Calculate correction power based on heading deviation
            double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double adjustmentError = initialHeading - currentHeading;

            // Normalize the adjustment error to range [-180, 180]
            if (adjustmentError > 180) adjustmentError -= 360;
            if (adjustmentError < -180) adjustmentError += 360;

            // Ramp down correction power similarly as the main currentPower
            double correctionRampDownFactor = currentPower / STRAFE_SPEED; // Same ramp factor as currentPower
            correctionPower = STRAFE_CORRECTION_GAIN * adjustmentError * correctionRampDownFactor;

            // Apply power to motors (strafe + correction)
            frontleftDrive.setPower(currentPower + correctionPower);  // Front left motor forward with correction
            frontrightDrive.setPower(-currentPower - correctionPower); // Front right motor backward with correction
            backleftDrive.setPower(-currentPower + correctionPower);   // Left back motor backward with correction
            backrightDrive.setPower(currentPower - correctionPower);   // Right back motor forward with correction
            telemetry.addData("CURRENT POWER", currentPower);
            telemetry.addData("CORRECTION POWER", correctionPower);
            telemetry.update();
        }

        // Stop the motors after the loop is done
        frontleftDrive.setPower(0);
        frontrightDrive.setPower(0);
        backleftDrive.setPower(0);
        backrightDrive.setPower(0);


        // Sleep for a short period (e.g., 100ms) to allow robot to stabilize before correction
        sleep(100);  // 100 milliseconds pause to let the robot settle

        // Now adjust the robot back to its initial heading if needed
        double finalHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double adjustmentError = initialHeading - finalHeading;

        // Normalize the adjustment error to range [-180, 180]
        if (adjustmentError > 180) adjustmentError -= 360;
        if (adjustmentError < -180) adjustmentError += 360;

        // Apply a correction until the robot is back to its initial heading
        while (opModeIsActive() && Math.abs(adjustmentError) > 1) { // Tolerance of 1 degree
            finalHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            adjustmentError = initialHeading - finalHeading;

            // Normalize the adjustment error to range [-180, 180]
            if (adjustmentError > 180) adjustmentError -= 360;
            if (adjustmentError < -180) adjustmentError += 360;

            double adjustmentCorrection = STRAFE_CORRECTION_GAIN * adjustmentError;

            // Apply correction to motors
            if (adjustmentError > 0) {
                // Robot is turning counter-clockwise, apply correction to rotate right
                frontleftDrive.setPower(adjustmentCorrection);
                frontrightDrive.setPower(-adjustmentCorrection);
                backleftDrive.setPower(adjustmentCorrection);
                backrightDrive.setPower(-adjustmentCorrection);
            } else {
                // Robot is turning clockwise, apply correction to rotate left
                frontleftDrive.setPower(-adjustmentCorrection);
                frontrightDrive.setPower(adjustmentCorrection);
                backleftDrive.setPower(-adjustmentCorrection);
                backrightDrive.setPower(adjustmentCorrection);
            }
        }

        // Stop the motors after correction is done
        frontleftDrive.setPower(0);
        frontrightDrive.setPower(0);
        backleftDrive.setPower(0);
        backrightDrive.setPower(0);

        // Sleep for the specified time
        sleep(sleepMS);
    }


    //strafe robot right a specific distance
    public void strafeRightMM(int distanceMM, int sleepMS) {
        //long timeToTravelMM = (long) ((distanceMM / STRAIFE_MM_SECOND) * 1000); // Time in milliseconds

        // Calculate ramp-up/down distance
        double rampDistanceMM = ((FORWARD_MIN_SPEED + FORWARD_SPEED) / 2) * STRAIFE_RAMP_TIME / 1000.0 * STRAIFE_MM_SECOND;
        // Ensure ramp distances don't exceed total distance
        double constantSpeedDistanceMM = Math.max(0, distanceMM - 2 * rampDistanceMM);
        // Calculate durations for each phase
        double rampTimeMS = STRAIFE_RAMP_TIME; // Ramp-up and ramp-down times are fixed
        double constantSpeedTimeMS = (constantSpeedDistanceMM / STRAIFE_MM_SECOND) * 1000;
        // Total time to travel
        long totalTimeMS = (long)(2 * rampTimeMS + constantSpeedTimeMS);
        strafeRight(totalTimeMS, sleepMS);
    }
    public void strafeRight(double milliseconds, int sleepMS) {
        long startTime = System.currentTimeMillis();
        double currentPower = STRAFE_MIN_SPEED;
        double correctionPower = 0; // Initialize correction power to 0
        runtime.reset();

        // Get the initial heading (yaw) from the IMU before the strafe
        double initialHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        while (opModeIsActive() && (runtime.milliseconds() < milliseconds)) {
            double elapsedTime = runtime.milliseconds();

            // Ramp up, full speed, and ramp down phases
            if (elapsedTime < STRAIFE_RAMP_TIME) {
                telemetry.addData("RAMPING", "UP");
                currentPower = STRAFE_MIN_SPEED + (STRAFE_SPEED - STRAFE_MIN_SPEED) * (elapsedTime / STRAIFE_RAMP_TIME);
            } else if (elapsedTime < (milliseconds - STRAIFE_RAMP_DOWN_TIME)) {
                telemetry.addData("RAMPING", "FULL");
                currentPower = STRAFE_SPEED;
            } else {
                telemetry.addData("RAMPING", "DOWN");
                double timeSinceRampDown = elapsedTime - (milliseconds - STRAIFE_RAMP_DOWN_TIME);
                double rampDownFactor = 1.0 - (timeSinceRampDown / STRAIFE_RAMP_DOWN_TIME);
                currentPower = currentPower * rampDownFactor;
            }

            // Ensure currentPower stays within valid range
            currentPower = Math.max(0, Math.min(currentPower, STRAFE_SPEED));

            // Calculate correction power based on heading deviation
            double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double adjustmentError = initialHeading - currentHeading;

            // Normalize the adjustment error to range [-180, 180]
            if (adjustmentError > 180) adjustmentError -= 360;
            if (adjustmentError < -180) adjustmentError += 360;

            // Ramp down correction power with currentPower
            double correctionRampDownFactor = currentPower / STRAFE_SPEED;
            correctionPower = STRAFE_CORRECTION_GAIN * adjustmentError * correctionRampDownFactor;

            // Apply power to motors (strafe + correction) for strafing right
            frontleftDrive.setPower(-currentPower + correctionPower);  // Front left motor backward with correction
            frontrightDrive.setPower(currentPower - correctionPower);  // Front right motor forward with correction
            backleftDrive.setPower(currentPower + correctionPower);   // Back left motor forward with correction
            backrightDrive.setPower(-currentPower - correctionPower); // Back right motor backward with correction

            // Telemetry for debugging
            telemetry.addData("CURRENT POWER", currentPower);
            telemetry.addData("CORRECTION POWER", correctionPower);
            telemetry.update();
        }

        // Stop the motors after the loop
        frontleftDrive.setPower(0);
        frontrightDrive.setPower(0);
        backleftDrive.setPower(0);
        backrightDrive.setPower(0);

        // Optional: Adjust back to the initial heading
        sleep(100); // Stabilize before heading correction
        double finalHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double adjustmentError = initialHeading - finalHeading;

        if (adjustmentError > 180) adjustmentError -= 360;
        if (adjustmentError < -180) adjustmentError += 360;

        while (opModeIsActive() && Math.abs(adjustmentError) > 1) { // Tolerance of 1 degree
            finalHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            adjustmentError = initialHeading - finalHeading;

            if (adjustmentError > 180) adjustmentError -= 360;
            if (adjustmentError < -180) adjustmentError += 360;

            double adjustmentCorrection = STRAFE_CORRECTION_GAIN * adjustmentError;

            // Apply correction to motors
            if (adjustmentError > 0) {
                frontleftDrive.setPower(adjustmentCorrection);
                frontrightDrive.setPower(-adjustmentCorrection);
                backleftDrive.setPower(adjustmentCorrection);
                backrightDrive.setPower(-adjustmentCorrection);
            } else {
                frontleftDrive.setPower(-adjustmentCorrection);
                frontrightDrive.setPower(adjustmentCorrection);
                backleftDrive.setPower(-adjustmentCorrection);
                backrightDrive.setPower(adjustmentCorrection);
            }
        }

        // Stop motors after correction
        frontleftDrive.setPower(0);
        frontrightDrive.setPower(0);
        backleftDrive.setPower(0);
        backrightDrive.setPower(0);

        // Sleep for the specified time
        sleep(sleepMS);

    }

    //turn robot left a specific degrees (from current position)
    public void turnLeft(double targetAngle, int sleepMS) {
        sleep(200);
        runtime.reset();

        // Initial heading from IMU
        double zeroAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double currentAngle;
        targetAngle += zeroAngle;
        double currentError;
        double lastError = 0;
        double derivative;
        double motorPower;

        while (opModeIsActive()) {
            // Get the current angle from the IMU
            currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            // Calculate the error between target and current angle
            currentError = targetAngle - currentAngle;

            // Normalize the error to the range [-180, 180]
            currentError = (currentError + 360) % 360; // Ensure positive range
            if (currentError > 180) currentError -= 360; // Wrap to [-180, 180]

            // Calculate derivative (rate of change of error)
            derivative = currentError - lastError;

            // Calculate motor power using proportional + derivative control
            motorPower = (Kp * currentError) + (Kd * derivative);

            // Clamp motor power to safe range
            motorPower = Range.clip(motorPower, -TURN_SPEED_MAX, TURN_SPEED_MAX);

            // Ensure minimum power for small errors (only if moving)
            if (Math.abs(motorPower) < TURN_SPEED_MIN && Math.abs(currentError) > ROTATE_ERROR_DEGREES) {
                motorPower = Math.copySign(TURN_SPEED_MIN, motorPower);
            }

            // Apply motor power for turning
            frontrightDrive.setPower(-motorPower);
            frontleftDrive.setPower(motorPower);
            backleftDrive.setPower(motorPower);
            backrightDrive.setPower(-motorPower);

            // Log telemetry for debugging
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Current Angle", currentAngle);
            telemetry.addData("Angle Error", currentError);
            telemetry.addData("Motor Power", motorPower);
            telemetry.update();

            // Break the loop if the error is within the acceptable range
            if (Math.abs(currentError) <= ROTATE_ERROR_DEGREES) break;

            // Update last error for derivative calculation
            lastError = currentError;

            // Small delay for control loop
            sleep(10);
        }

        // Stop the motors after turning
        frontleftDrive.setPower(0);
        frontrightDrive.setPower(0);
        backleftDrive.setPower(0);
        backrightDrive.setPower(0);

        // Optional delay after turning
        sleep(sleepMS);
    }

   /* //turn robot right a specific degrees (from current position)
    public void turnRight(double targetAngle, int sleepMS) {
        sleep(200);
        runtime.reset();

        // Initial heading from IMU
        double zeroAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double currentAngle;
        targetAngle += (zeroAngle*-1); // Subtract targetAngle for right turn
        double currentError;
        double lastError = 0;
        double derivative;
        double motorPower;

        while (opModeIsActive()) {
            // Get the current angle from the IMU
            currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            // Calculate the error between target and current angle
            currentError = targetAngle - currentAngle;

            // Normalize the error to the range [-180, 180]
            currentError = (currentError + 360) % 360; // Ensure positive range
            if (currentError > 180) currentError -= 360; // Wrap to [-180, 180]

            // Calculate derivative (rate of change of error)
            derivative = currentError - lastError;

            // Calculate motor power using proportional + derivative control
            motorPower = (Kp * currentError) + (Kd * derivative);

            // Clamp motor power to safe range
            motorPower = Range.clip(motorPower, -TURN_SPEED_MAX, TURN_SPEED_MAX);

            // Ensure minimum power for small errors (only if moving)
            if (Math.abs(motorPower) < TURN_SPEED_MIN && Math.abs(currentError) > ROTATE_ERROR_DEGREES) {
                motorPower = Math.copySign(TURN_SPEED_MIN, motorPower);
            }

            // Apply motor power for turning (reverse directions for turning right)
            frontrightDrive.setPower(motorPower);
            frontleftDrive.setPower(-motorPower);
            backleftDrive.setPower(-motorPower);
            backrightDrive.setPower(motorPower);

            // Log telemetry for debugging
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Current Angle", currentAngle);
            telemetry.addData("Angle Error", currentError);
            telemetry.addData("Motor Power", motorPower);
            telemetry.update();

            // Break the loop if the error is within the acceptable range
            if (Math.abs(currentError) <= ROTATE_ERROR_DEGREES) break;

            // Update last error for derivative calculation
            lastError = currentError;

            // Small delay for control loop
            sleep(10);
        }

        // Stop the motors after turning
        frontleftDrive.setPower(0);
        frontrightDrive.setPower(0);
        backleftDrive.setPower(0);
        backrightDrive.setPower(0);

        // Optional delay after turning
        sleep(sleepMS);
    }
*/

    public void turnRight(double targetAngle, int sleepMS) {
        sleep(200);
        runtime.reset();

        // Initial heading from IMU
        double zeroAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double currentAngle;
        double targetAbsoluteAngle = zeroAngle - targetAngle; // Subtract the targetAngle for a right turn
        double currentError;
        double lastError = 0;
        double derivative;
        double motorPower;

        while (opModeIsActive()) {
            // Get the current angle from the IMU
            currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            // Calculate the error between target and current angle
            currentError = targetAbsoluteAngle - currentAngle;
            // Normalize the error to the range [-180, 180]
            currentError = (currentError + 360) % 360; // Ensure positive range
            if (currentError > 180) currentError -= 360; // Wrap to [-180, 180]
            // If error is negative, normalize for turning right
            if (currentError > 0) currentError -= 360;
            // Calculate derivative (rate of change of error)
            derivative = currentError - lastError;
            // Calculate motor power using proportional + derivative control
            motorPower = (Kp * currentError) + (Kd * derivative);
            // Clamp motor power to safe range
            motorPower = Range.clip(motorPower, -TURN_SPEED_MAX, TURN_SPEED_MAX);
            // Ensure minimum power for small errors (only if moving)
            if (Math.abs(motorPower) < TURN_SPEED_MIN && Math.abs(currentError) > ROTATE_ERROR_DEGREES) {
                motorPower = Math.copySign(TURN_SPEED_MIN, motorPower);
            }

            // Apply motor power for turning
            frontrightDrive.setPower(motorPower); // Positive power for turning right
            frontleftDrive.setPower(-motorPower); // Negative power for turning right
            backleftDrive.setPower(-motorPower);  // Negative power for turning right
            backrightDrive.setPower(motorPower);  // Positive power for turning right

            // Log telemetry for debugging
            telemetry.addData("Target Absolute Angle", targetAbsoluteAngle);
            telemetry.addData("Current Angle", currentAngle);
            telemetry.addData("Angle Error", currentError);
            telemetry.addData("Motor Power", motorPower);
            telemetry.update();

            // Break the loop if the error is within the acceptable range
            if (Math.abs(currentError) <= ROTATE_ERROR_DEGREES) break;
            // Update last error for derivative calculation
            lastError = currentError;
            // Small delay for control loop
            sleep(10);
        }

        // Stop the motors after turning
        frontleftDrive.setPower(0);
        frontrightDrive.setPower(0);
        backleftDrive.setPower(0);
        backrightDrive.setPower(0);

        // Optional delay after turning
        sleep(sleepMS);
    }


    //raise or lower the robot's arm to a specific height
    public void moveArm(int targetPosition, int actionTimeout, int sleepMS) {
        long actionStartTime;
        int currentArmPosition = armMotor.getCurrentPosition();
        int currentExtensionPosition = extensionArmMotor.getCurrentPosition();

        if (currentArmPosition < targetPosition) {
            // Step 1: Retract the extension
            extensionTargetPosition = EXTENSION_MIN_POSITION;
            extensionArmMotor.setTargetPosition(extensionTargetPosition); // Fully retract
            extensionArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            currentExtensionPower = calcExtensionPower();
            extensionArmMotor.setPower(EXTENSION_BASE_POWER); // Set appropriate power
            actionStartTime = System.currentTimeMillis();
            while (opModeIsActive() && extensionArmMotor.isBusy() && (System.currentTimeMillis() - actionStartTime) < actionTimeout) {
                addTelemetry();
                if (Math.abs(extensionArmMotor.getCurrentPosition() - extensionTargetPosition) < MOTOR_TOLERANCE) {
                    break; // Break if within tolerance
                }
            }

            // Step 2: Move the arm to the target height
            armTargetPosition = targetPosition;
            armMotor.setTargetPosition(armTargetPosition);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            currentArmPower = calcArmPower();
            armMotor.setPower(currentArmPower);
            actionStartTime = System.currentTimeMillis();
            while (opModeIsActive() && armMotor.isBusy() && (System.currentTimeMillis() - actionStartTime) < actionTimeout) {

                addTelemetry();
                if (Math.abs(armMotor.getCurrentPosition() - armTargetPosition) < MOTOR_TOLERANCE) {
                    break; // Break if within tolerance
                }
            }
        } else {
            // If the arm is already at or above the target height
            // Step 1: Move the arm to the target height
            armTargetPosition = targetPosition;
            armMotor.setTargetPosition(armTargetPosition);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            currentArmPower = calcArmPower();
            armMotor.setPower(currentArmPower);
            actionStartTime = System.currentTimeMillis();
            while (opModeIsActive() && armMotor.isBusy() && (System.currentTimeMillis() - actionStartTime) < actionTimeout) {
                addTelemetry();
                if (Math.abs(armMotor.getCurrentPosition() - armTargetPosition) < MOTOR_TOLERANCE) {
                    break; // Break if within tolerance
                }
            }
        }

        // Stop motors at the end to ensure no lingering movement
        armMotor.setPower(ARM_BASE_POWER);
        extensionArmMotor.setPower(0);
        sleep(sleepMS);
    }


    //raise or lower the robot's arm to a specific height
    public void moveArmEncoder(int targetPosition, int actionTimeout, int sleepMS) {
        long actionStartTime;
        int startArmPosition = armMotor.getCurrentPosition();
        double currentArmSpeed = 0;

        if (startArmPosition < targetPosition) {
            // Step 1: Retract the extension
            moveExtensionEncoder(0,1500,200);

            // Step 2: Move the arm to the target height
            armTargetPosition = targetPosition;
            armMotor.setTargetPosition(armTargetPosition);
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            currentArmSpeed = ARM_MIN_SPEED;
            armMotor.setPower(currentArmSpeed);

            int totalTicks = Math.abs(targetPosition - armMotor.getCurrentPosition()); // Total ticks for the move
            while (opModeIsActive() && armMotor.isBusy()) {
                int currentPos = armMotor.getCurrentPosition();
                int distanceToTarget = Math.abs(targetPosition - currentPos);
                int distanceFromStart = Math.abs(currentPos - startArmPosition);

                // Calculate the arm power based on ramping up or down
                if (distanceFromStart < ARM_RAMP_TICKS) {
                    // State 1: Ramp-Up (when within the first XXX ticks of movement)
                    //currentArmPower = ARM_ENCODER_SPEED * (traveledTicks / (double) ARM_RAMP_TICKS);
                    //currentArmSpeed = ARM_MIN_SPEED + (ARM_MAX_SPEED - ARM_MIN_SPEED) * ((float) distanceFromStart / ARM_RAMP_TICKS);
                    float rampFactor = Math.min(1.0f, Math.max(0.0f, (float) distanceFromStart / ARM_RAMP_TICKS));
                    currentArmSpeed = ARM_MIN_SPEED + (ARM_MAX_SPEED - ARM_MIN_SPEED) * rampFactor;
                    // Ensure power doesn't exceed EXTENSION_MAX_SPEED or drop below EXTENSION_MIN_SPEED
                    currentArmSpeed = Math.max(ARM_MIN_SPEED, Math.min(currentArmSpeed, ARM_MAX_SPEED));
                }
                else if (distanceFromStart > ARM_RAMP_TICKS && distanceToTarget > ARM_RAMP_TICKS) {
                    // State 2: Full Speed (between the first and last 50 ticks)
                    currentArmSpeed = ARM_MAX_SPEED;
                    //currentArmPower = ARM_ENCODER_SPEED * (remainingTicks / (double) ARM_RAMP_TICKS);
                }
                else if (distanceToTarget <= ARM_RAMP_TICKS) {
                    // State 3: Ramp-Down (when within the last XXX ticks of movement)
                    //currentArmSpeed = ARM_MIN_SPEED + (ARM_MAX_SPEED - ARM_MIN_SPEED) * ((float) distanceToTarget / ARM_RAMP_TICKS);
                    float rampFactor = Math.min(1.0f, Math.max(0.0f, (float) distanceToTarget / ARM_RAMP_TICKS));
                    // Interpolate speed using the ramp factor
                    currentArmSpeed = ARM_MIN_SPEED + (ARM_MAX_SPEED - ARM_MIN_SPEED) * rampFactor;
                    // Ensure power doesn't exceed EXTENSION_MAX_SPEED or drop below EXTENSION_MIN_SPEED
                    currentArmSpeed = Math.max(ARM_MIN_SPEED, Math.min(currentArmSpeed, ARM_MIN_SPEED));
                }
                armMotor.setPower(currentArmSpeed);
                addTelemetry();

                if (distanceToTarget < MOTOR_TOLERANCE) {
                    break;
                }
            }
            armMotor.setPower(ARM_MIN_SPEED);

        } else {
            // If the arm is already at or above the target height
            // Step 1: Move the arm to the target height
            armTargetPosition = targetPosition;
            armMotor.setTargetPosition(armTargetPosition);
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            int totalTicks = Math.abs(targetPosition - armMotor.getCurrentPosition()); // Total ticks for the move
            while (opModeIsActive() && armMotor.isBusy()) {
                int currentPos = armMotor.getCurrentPosition();
                int distanceToTarget = Math.abs(targetPosition - currentPos);
                int distanceFromStart = Math.abs(currentPos - startArmPosition);

                // Calculate the arm power based on ramping up or down
                if (distanceFromStart < ARM_RAMP_TICKS) {
                    // State 1: Ramp-Up (when within the first XXX ticks of movement)
                    //currentArmPower = ARM_ENCODER_SPEED * (traveledTicks / (double) ARM_RAMP_TICKS);
                    //currentArmSpeed = ARM_MIN_SPEED + (ARM_MAX_SPEED - ARM_MIN_SPEED) * ((float) distanceFromStart / ARM_RAMP_TICKS);
                    float rampFactor = Math.min(1.0f, Math.max(0.0f, (float) distanceFromStart / ARM_RAMP_TICKS));
                    currentArmSpeed = ARM_MIN_SPEED + (ARM_MAX_SPEED - ARM_MIN_SPEED) * rampFactor;
                    // Ensure power doesn't exceed EXTENSION_MAX_SPEED or drop below EXTENSION_MIN_SPEED
                    currentArmSpeed = Math.max(ARM_MIN_SPEED, Math.min(currentArmSpeed, ARM_MAX_SPEED));
                }
                else if (distanceFromStart > ARM_RAMP_TICKS && distanceToTarget > ARM_RAMP_TICKS) {
                    // State 2: Full Speed (between the first and last 50 ticks)
                    currentArmSpeed = ARM_MAX_SPEED;
                    //currentArmPower = ARM_ENCODER_SPEED * (remainingTicks / (double) ARM_RAMP_TICKS);
                }
                else if (distanceToTarget <= ARM_RAMP_TICKS) {
                    // State 3: Ramp-Down (when within the last XXX ticks of movement)
                    //currentArmSpeed = ARM_MIN_SPEED + (ARM_MAX_SPEED - ARM_MIN_SPEED) * ((float) distanceToTarget / ARM_RAMP_TICKS);
                    float rampFactor = Math.min(1.0f, Math.max(0.0f, (float) distanceToTarget / ARM_RAMP_TICKS));
                    // Interpolate speed using the ramp factor
                    currentArmSpeed = ARM_MIN_SPEED + (ARM_MAX_SPEED - ARM_MIN_SPEED) * rampFactor;
                    // Ensure power doesn't exceed EXTENSION_MAX_SPEED or drop below EXTENSION_MIN_SPEED
                    currentArmSpeed = Math.max(ARM_MIN_SPEED, Math.min(currentArmSpeed, ARM_MIN_SPEED));
                }
                armMotor.setPower(currentArmSpeed);
                addTelemetry();

                if (distanceToTarget < MOTOR_TOLERANCE) {
                    break;
                }
            }
        }

        // Stop motors at the end to ensure no lingering movement
        armMotor.setPower(ARM_MIN_SPEED);
        extensionArmMotor.setPower(0);
        sleep(sleepMS);
    }

    //move the extension out to a specific position
    public void moveExtension(int targetPosition, int actionTimeout, int sleepMS) {
        long actionStartTime;
        extensionTargetPosition = targetPosition;
        extensionArmMotor.setTargetPosition(extensionTargetPosition);
        extensionArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        currentExtensionPower = calcExtensionPower();
        extensionArmMotor.setPower(EXTENSION_BASE_POWER);
        actionStartTime = System.currentTimeMillis();
        while (extensionArmMotor.isBusy() && (System.currentTimeMillis() - actionStartTime) < actionTimeout) {
            if (Math.abs(extensionArmMotor.getCurrentPosition() - extensionTargetPosition) < MOTOR_TOLERANCE) {
                break; // Break if within tolerance
            }
        }
        sleep(sleepMS);
    }

    public void moveExtensionEncoder(int targetPosition, int actionTimeout, int sleepMS) {
        long actionStartTime;
        double currentExtensionSpeed = 0;
        int startExtensionPosition = extensionArmMotor.getCurrentPosition();
        extensionTargetPosition = targetPosition;
        extensionArmMotor.setTargetPosition(extensionTargetPosition);
        extensionArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        currentExtensionSpeed = EXTENSION_MIN_SPEED;
        extensionArmMotor.setPower(currentExtensionSpeed);
        actionStartTime = System.currentTimeMillis();
        while (extensionArmMotor.isBusy() && (System.currentTimeMillis() - actionStartTime) < actionTimeout) {

            int totalTicks = Math.abs(targetPosition - extensionArmMotor.getCurrentPosition()); // Total ticks for the move
            while (opModeIsActive() && extensionArmMotor.isBusy()) {
                int currentPos = armMotor.getCurrentPosition();
                int distanceToTarget = Math.abs(targetPosition - currentPos);
                int distanceFromStart = Math.abs(currentPos - startExtensionPosition);

                // Calculate the arm power based on ramping up or down
                if (distanceFromStart < EXTENSION_RAMP_TICKS) {
                    // State 1: Ramp-Up (when within the first XXX ticks of movement)
                    //currentArmPower = ARM_ENCODER_SPEED * (traveledTicks / (double) ARM_RAMP_TICKS);
                    //currentExtensionSpeed = EXTENSION_MIN_SPEED + (ARM_MAX_SPEED - EXTENSION_MIN_SPEED) * ((float) distanceFromStart / EXTENSION_RAMP_TICKS);
                    // Cap the ramp factor to [0, 1] to avoid exceeding bounds
                    float rampFactor = Math.min(1.0f, Math.max(0.0f, (float) distanceFromStart / EXTENSION_RAMP_TICKS));
                    currentExtensionSpeed = EXTENSION_MIN_SPEED + (EXTENSION_MAX_SPEED - EXTENSION_MIN_SPEED) * rampFactor;
                    // Ensure power doesn't exceed EXTENSION_MAX_SPEED or drop below EXTENSION_MIN_SPEED
                    currentExtensionSpeed = Math.max(EXTENSION_MIN_SPEED, Math.min(currentExtensionSpeed, EXTENSION_MAX_SPEED));

                }
                else if (distanceFromStart > EXTENSION_RAMP_TICKS && distanceToTarget > EXTENSION_RAMP_TICKS) {
                    // State 2: Full Speed (between the first and last 50 ticks)
                    currentExtensionSpeed = EXTENSION_MAX_SPEED;
                    //currentArmPower = ARM_ENCODER_SPEED * (remainingTicks / (double) ARM_RAMP_TICKS);
                }
                else if (distanceToTarget <= EXTENSION_RAMP_TICKS) {
                    // State 3: Ramp-Down (when within the last XXX ticks of movement)
                    //currentExtensionSpeed = EXTENSION_MIN_SPEED + (EXTENSION_MAX_SPEED - EXTENSION_MIN_SPEED) * ((float) distanceToTarget / EXTENSION_RAMP_TICKS);
                    // Calculate ramp factor for the ramp-down phase
                    float rampFactor = Math.min(1.0f, Math.max(0.0f, (float) distanceToTarget / EXTENSION_RAMP_TICKS));
                    // Interpolate speed using the ramp factor
                    currentExtensionSpeed = EXTENSION_MIN_SPEED + (EXTENSION_MAX_SPEED - EXTENSION_MIN_SPEED) * rampFactor;
                    // Ensure power doesn't exceed EXTENSION_MAX_SPEED or drop below EXTENSION_MIN_SPEED
                    currentExtensionSpeed = Math.max(EXTENSION_MIN_SPEED, Math.min(currentExtensionSpeed, EXTENSION_MAX_SPEED));

                }
                extensionArmMotor.setPower(currentExtensionSpeed);
                addTelemetry();

                if (distanceToTarget < MOTOR_TOLERANCE) {
                    break;
                }
            }
        }
        sleep(sleepMS);
    }

    public void intakeClaw(int runMS, int sleepMS) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        leftWheelServo.setPosition(SERVO_BACKWARD);  // Spin inward
        rightWheelServo.setPosition(SERVO_FORWARD); // Spin inward
        // Wait for the specified duration
        while (opModeIsActive() && timer.milliseconds() < runMS) {
            telemetry.addData("Intake Claw", "Running for %d ms", runMS);
            telemetry.update();
        }
        leftWheelServo.setPosition(SERVO_STOPPED);  // Neutral
        rightWheelServo.setPosition(SERVO_STOPPED); // Neutral
        sleep(sleepMS);
    }

    public void outputClaw(int runMS, int sleepMS) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        leftWheelServo.setPosition(SERVO_FORWARD);  // Spin inward
        rightWheelServo.setPosition(SERVO_BACKWARD); // Spin inward
        // Wait for the specified duration
        while (opModeIsActive() && timer.milliseconds() < runMS) {
            telemetry.addData("Output Claw", "Running for %d ms", runMS);
            telemetry.update();
        }
        leftWheelServo.setPosition(SERVO_STOPPED);  // Neutral
        rightWheelServo.setPosition(SERVO_STOPPED); // Neutral
        sleep(sleepMS);
    }

    //shut down the robot (end of auto)
    public void closeRobot() {
        moveExtension(0,1000,100);
        moveArm(0,1000,100);
    }

    //calculate the extension power based on the current position of the extension
    public double calcExtensionPower() {
        //Adjust motor power based on proximity to limits - only to be used in runToPosition
        int currentPosition = extensionArmMotor.getCurrentPosition();
        double extensionProximityFactor = 1.0; // Full power by default
        if (currentPosition < extensionTargetPosition) {
            // Arm is moving towards the max position
            if (extensionTargetPosition - currentPosition < 50) {
                extensionProximityFactor = (extensionTargetPosition - currentPosition) / 50.0; // Scale based on distance to target
            }
        } else if (currentPosition > extensionTargetPosition) {
            // Arm is moving towards the min position
            if (currentPosition - extensionTargetPosition < 50) {
                extensionProximityFactor = (currentPosition - extensionTargetPosition) / 50.0; // Scale based on distance to target
            }
        }

        //Calculate the power dynamically
        double extensionPower = EXTENSION_BASE_POWER + ((double) armTargetPosition / ARM_MAX_POSITION) * EXTENSION_EXTRA_FORCE;

        extensionPower = extensionPower * extensionProximityFactor; // Apply proximity factor to the power
        extensionPower = Range.clip(extensionPower, 0.2, 1.0); // Limit power to safe values
        return extensionPower;
    }

    //calculate the power of the arm
    public double calcArmPower() {
        // Adjust arm motor power based on vertical arm position - only to be used in runToPosition
        double horizontalArmPower = ARM_BASE_POWER;
        double verticalFactor = (double) extensionTargetPosition / EXTENSION_MAX_POSITION;
        horizontalArmPower += verticalFactor * ARM_EXTRA_FORCE; // Add extra power when fully raised
        horizontalArmPower = Range.clip(horizontalArmPower, 0.2, 1.0);
        return horizontalArmPower;
    }

    //get the current angle of the robot
    public double getAngle() {
        double angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        return angle;
    }

    private void addTelemetry() {
        // Log telemetry for debugging
        telemetry.addData("Arm Target Position", armTargetPosition);
        telemetry.addData("Arm Current Postiion", armMotor.getCurrentPosition());
        telemetry.addData("Arm Current Power", currentArmPower);
        telemetry.addData("Extension Target Position", extensionTargetPosition);
        telemetry.addData("Extension Current Postiion", extensionArmMotor.getCurrentPosition());
        telemetry.addData("Extension Current Power", currentExtensionPower);
        telemetry.update();
    }
}
