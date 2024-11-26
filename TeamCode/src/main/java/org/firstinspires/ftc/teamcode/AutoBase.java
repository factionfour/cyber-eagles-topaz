package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

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

    // Arm motor limits and power
    int armTargetPosition = 0;
    int ARM_MIN_POSITION =100;    // Minimum encoder position (fully retracted)
    int ARM_MAX_POSITION = 440; // Maximum encoder position (fully extended)
    double ARM_BASE_POWER = 0.3;
    double ARM_EXTRA_FORCE = 0.6;
    int ARM_MIN_SPEED = 12; // How far to move per loop iteration
    int ARM_MIN_POSITION_WHEN_EXTENSION_EXTENDED = 150;

    // Vertical extension limits and base power
    int extensionTargetPosition = 0;
    int EXTENSION_MIN_POSITION = 0;    // Minimum height (fully lowered)
    int EXTENSION_MAX_POSITION = 2200; // Maximum height (fully raised)

    int EXTENSION_MIN_SPEED = 80; // How far to move per loop iteration
    double EXTENSION_BASE_POWER = 0.5; // Base power to hold position
    double EXTENSION_EXTRA_FORCE = 0.4; // Extra power when arm is extended
    long DEBOUNCE_DELAY = 40; // Time in milliseconds to wait before accepting another press
    int MOTOR_TOLERANCE = 10; // Acceptable error in encoder ticks

    //servo limits
    double SERVO_STOPPED = 0.5;
    double SERVO_FORWARD = 1;
    double SERVO_BACKWARD = 0;

    double FORWARD_MIN_SPEED = 0.4;
    double FORWARD_SPEED = 1;
    int FORWARD_RAMP_TIME = 200;
    double TURN_SPEED    = 1;

    int RAMP_TIME = 200;

    //pre-defined positions
    int HOOK_EXTENSION_POSITION = 1800;
    int HOOK_ARM_HEIGHT = 420;
    int HOOK_RELEASE_EXTENSION_POSITION = 1500;
    int HOOK_RELEASE_ARM_HEIGHT = 350;

    int dynamicArmMinPosition = 0;
    double currentExtensionPower = 0;
    double currentArmPower = 0;
    boolean predefinedActionRunning = false;

    //CONVERSION TO CENTIMETERS
    double FORWARD_MM_SECOND = 1040;
    double BACKWARD_MM_SECOND = 1060;
    double STRAIFE_MM_SECOND = 706;
    final double STRAFE_FACTOR = 1.1;

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
        armMotor.setTargetPosition(ARM_MIN_POSITION);
        armMotor.setPower(ARM_BASE_POWER);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armTargetPosition = ARM_MIN_POSITION;

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

        telemetry.addData(">", "Charlie 2 is READY for auto mode.  Press START.");
        waitForStart();
    }



    public void driveForwardMM(int distanceMM, int sleepMS) {
        long timeToTravelMM = (long) ((distanceMM / FORWARD_MM_SECOND) * 1000); // Time in milliseconds
        driveForward(timeToTravelMM,sleepMS);
    }
    public void driveForward(double milliseconds, int sleepMS) {
        long startTime = System.currentTimeMillis();
        double currentPower = FORWARD_MIN_SPEED;
        runtime.reset();

        while (opModeIsActive() && (runtime.milliseconds() < milliseconds)) {
            double elapsedTime = runtime.milliseconds();

            if (elapsedTime < (milliseconds - FORWARD_RAMP_TIME)) {
                // Ramp up phase
                currentPower = FORWARD_MIN_SPEED + (FORWARD_SPEED - FORWARD_MIN_SPEED) * (elapsedTime / (double)(milliseconds - FORWARD_RAMP_TIME));
            } else {
                // Ramp down phase (last FORWARD_RAMP_TIME milliseconds)
                double timeElapsedSinceRampDown = elapsedTime - (milliseconds - FORWARD_RAMP_TIME);
                currentPower = FORWARD_SPEED - (FORWARD_SPEED * (timeElapsedSinceRampDown / (double)FORWARD_RAMP_TIME));
            }

            // Ensure currentPower stays within the range [FORWARD_MIN_SPEED, FORWARD_SPEED]
            currentPower = Math.max(FORWARD_MIN_SPEED, Math.min(currentPower, FORWARD_SPEED));

            // Apply the power to the motors (inverting the power for right motors)
            frontrightDrive.setPower(currentPower);
            frontleftDrive.setPower(currentPower);
            backleftDrive.setPower(currentPower);
            backrightDrive.setPower(currentPower);

            // Telemetry for monitoring
            telemetry.addData("Wheel Power", currentPower);
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

    public void driveBackwardMM(int distanceMM, int sleepMS) {
        long timeToTravelMM = (long) ((distanceMM / FORWARD_MM_SECOND) * 1000); // Time in milliseconds
        driveBackward(timeToTravelMM,sleepMS);
    }

    public void driveBackward(double milliseconds, int sleepMS) {
        long startTime = System.currentTimeMillis();
        double currentPower = FORWARD_MIN_SPEED;
        runtime.reset();

        while (opModeIsActive() && (runtime.milliseconds() < milliseconds)) {
            double elapsedTime = runtime.milliseconds();

            if (elapsedTime < (milliseconds - FORWARD_RAMP_TIME)) {
                // Ramp up phase
                currentPower = FORWARD_MIN_SPEED + (FORWARD_SPEED - FORWARD_MIN_SPEED) * (elapsedTime / (double)(milliseconds - FORWARD_RAMP_TIME));
            } else {
                // Ramp down phase (last FORWARD_RAMP_TIME milliseconds)
                double timeElapsedSinceRampDown = elapsedTime - (milliseconds - FORWARD_RAMP_TIME);
                currentPower = FORWARD_SPEED - (FORWARD_SPEED * (timeElapsedSinceRampDown / (double)FORWARD_RAMP_TIME));
            }

            // Ensure currentPower stays within the range [FORWARD_MIN_SPEED, FORWARD_SPEED]
            currentPower = Math.max(FORWARD_MIN_SPEED, Math.min(currentPower, FORWARD_SPEED));

            // Apply the power to the motors (inverting the power for right motors)
            frontrightDrive.setPower(-currentPower);
            frontleftDrive.setPower(-currentPower);
            backleftDrive.setPower(-currentPower);
            backrightDrive.setPower(-currentPower);

            // Telemetry for monitoring
            telemetry.addData("Wheel Power", currentPower);
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

    public void strafeLeftMM(int distanceMM, int sleepMS) {
        long timeToTravelMM = (long) ((distanceMM / STRAIFE_MM_SECOND) * 1000); // Time in milliseconds
        strafeLeft(timeToTravelMM, sleepMS);
    }

    public void strafeLeft(double milliseconds, int sleepMS) {
        long startTime = System.currentTimeMillis();
        double currentPower = FORWARD_MIN_SPEED;
        runtime.reset();

        while (opModeIsActive() && (runtime.milliseconds() < milliseconds)) {
            double elapsedTime = runtime.milliseconds();

            if (elapsedTime < (milliseconds - FORWARD_RAMP_TIME)) {
                // Ramp up phase
                currentPower = FORWARD_MIN_SPEED + (FORWARD_SPEED - FORWARD_MIN_SPEED) * (elapsedTime / (double)(milliseconds - FORWARD_RAMP_TIME));
            } else {
                // Ramp down phase (last FORWARD_RAMP_TIME milliseconds)
                double timeElapsedSinceRampDown = elapsedTime - (milliseconds - FORWARD_RAMP_TIME);
                currentPower = FORWARD_SPEED - (FORWARD_SPEED * (timeElapsedSinceRampDown / (double)FORWARD_RAMP_TIME));
            }

            // Ensure currentPower stays within the range [FORWARD_MIN_SPEED, FORWARD_SPEED]
            currentPower = Math.max(FORWARD_MIN_SPEED, Math.min(currentPower, FORWARD_SPEED));

            // Apply the power to the motors (inverting the power for right motors)
            frontrightDrive.setPower(-currentPower);
            frontleftDrive.setPower(currentPower);
            backleftDrive.setPower(-currentPower);
            backrightDrive.setPower(currentPower);

            // Telemetry for monitoring
            telemetry.addData("Wheel Power", currentPower);
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

    public void strafeRightMM(int distanceMM, int sleepMS) {
        long timeToTravelMM = (long) ((distanceMM / STRAIFE_MM_SECOND) * 1000); // Time in milliseconds
        strafeRight(timeToTravelMM, sleepMS);
    }
    public void strafeRight(double milliseconds, int sleepMS) {
        long startTime = System.currentTimeMillis();
        double currentPower = FORWARD_MIN_SPEED;
        runtime.reset();

        while (opModeIsActive() && (runtime.milliseconds() < milliseconds)) {
            double elapsedTime = runtime.milliseconds();

            if (elapsedTime < (milliseconds - FORWARD_RAMP_TIME)) {
                // Ramp up phase
                currentPower = FORWARD_MIN_SPEED + (FORWARD_SPEED - FORWARD_MIN_SPEED) * (elapsedTime / (double)(milliseconds - FORWARD_RAMP_TIME));
            } else {
                // Ramp down phase (last FORWARD_RAMP_TIME milliseconds)
                double timeElapsedSinceRampDown = elapsedTime - (milliseconds - FORWARD_RAMP_TIME);
                currentPower = FORWARD_SPEED - (FORWARD_SPEED * (timeElapsedSinceRampDown / (double)FORWARD_RAMP_TIME));
            }

            // Ensure currentPower stays within the range [FORWARD_MIN_SPEED, FORWARD_SPEED]
            currentPower = Math.max(FORWARD_MIN_SPEED, Math.min(currentPower, FORWARD_SPEED));

            // Apply the power to the motors (inverting the power for right motors)
            frontrightDrive.setPower(currentPower);
            frontleftDrive.setPower(-currentPower);
            backleftDrive.setPower(currentPower);
            backrightDrive.setPower(-currentPower);

            // Telemetry for monitoring
            telemetry.addData("Wheel Power", currentPower);
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

    public void turnLeft(double milliseconds, int sleepMS) {
        frontrightDrive.setPower(-FORWARD_SPEED);
        frontleftDrive.setPower(FORWARD_SPEED);
        backleftDrive.setPower(FORWARD_SPEED);
        backrightDrive.setPower(-FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.milliseconds() < milliseconds)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.milliseconds());
            telemetry.update();
        }
        frontleftDrive.setPower(0);
        frontrightDrive.setPower(0);
        backleftDrive.setPower(0);
        backrightDrive.setPower(0);
        sleep(sleepMS);
    }
    public void turnRight(double milliseconds, int sleepMS) {
        frontrightDrive.setPower(FORWARD_SPEED);
        frontleftDrive.setPower(-FORWARD_SPEED);
        backleftDrive.setPower(-FORWARD_SPEED);
        backrightDrive.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.milliseconds() < milliseconds)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.milliseconds());
            telemetry.update();
        }
        frontleftDrive.setPower(0);
        frontrightDrive.setPower(0);
        backleftDrive.setPower(0);
        backrightDrive.setPower(0);
        sleep(sleepMS);
    }

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
            while (extensionArmMotor.isBusy() && (System.currentTimeMillis() - actionStartTime) < actionTimeout) {
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
            while (armMotor.isBusy() && (System.currentTimeMillis() - actionStartTime) < actionTimeout) {
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
            while (armMotor.isBusy() && (System.currentTimeMillis() - actionStartTime) < actionTimeout) {
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

    public void closeRobot() {
        moveExtension(0,1000,100);
        moveArm(0,1000,100);
    }

    public double calcExtensionPower() {
        //Adjust motor power based on proximity to limits
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

    public double calcArmPower() {
        // Adjust arm motor power based on vertical arm position
        double horizontalArmPower = ARM_BASE_POWER;
        double verticalFactor = (double) extensionTargetPosition / EXTENSION_MAX_POSITION;
        horizontalArmPower += verticalFactor * ARM_EXTRA_FORCE; // Add extra power when fully raised
        horizontalArmPower = Range.clip(horizontalArmPower, 0.2, 1.0);
        return horizontalArmPower;
    }
}
