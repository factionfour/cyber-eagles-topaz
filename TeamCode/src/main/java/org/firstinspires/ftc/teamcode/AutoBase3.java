package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public abstract class AutoBase3 extends LinearOpMode {
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

    // Arm motor limits
    int armTargetPosition = 0;
    int ARM_MIN_POSITION = 100;    // Minimum encoder position (fully retracted)
    int ARM_MAX_POSITION = 850; // Maximum encoder position (fully extended)

    //arm power (speed for encoders ONLY)
    int ARM_RAMP_TICKS = 50; // How far to move per loop iteration
    double ARM_MAX_SPEED = 0.7;
    double ARM_MIN_SPEED = 0.3;

    //extension limits and base power (both power and speed for encoders)
    int extensionTargetPosition = 0;
    int EXTENSION_MIN_POSITION = 0;    // Minimum height (fully lowered)
    int EXTENSION_MAX_POSITION = 2200; // Maximum height (fully raised)

    //extension power (speed for encoders ONLY)
    double EXTENSION_MIN_SPEED = 0.3;
    double EXTENSION_MAX_SPEED = 0.7;
    int EXTENSION_RAMP_TICKS = 75; // How far to move per loop iteration

    //motor position tolerances - used by encoders
    int MOTOR_TOLERANCE = 10; // Acceptable error in encoder ticks (used for arm & extension)

    //servo limits
    double SERVO_STOPPED = 0.5;
    double SERVO_FORWARD = 1;
    double SERVO_BACKWARD = 0;

    //pre-defined positions
    int TRAVEL_ARM_HEIGHT = 250;
    int TRAVEL_EXTENSION_HEIGHT = 200;

    int HOOK_EXTENSION_POSITION = 1800;
    int HOOK_ARM_HEIGHT = 750;
    int HOOK_RELEASE_ARM_HEIGHT = 670;
    int HOOK_RELEASE_EXTENSION_POSITION = 1600;

    int PICKUP_SPECIMEN_EXTENSION_POSITION = 1300;
    int PICKUP_SPECIMEN_ARM_HEIGHT = 130;

    int PARK_ARM_HEIGHT = 600;
    int PARK_EXTENSION_POSITION = 1400;

    double currentExtensionPower = 0;
    double currentArmPower = 0;


    //STRAFE MOVEMENT CONTROLS
    double STRAFE_MIN_SPEED = 0.4;
    double STRAFE_SPEED = 0.8;
    double STRAFE_RAMP_PERCENTAGE = .1;
    double STRAFE_CORRECTION_GAIN = 0.10;

    //FORWARD & BACKWARD MOVEMENT CONTROLS
    double FORWARD_MIN_SPEED = 0.1;
    double FORWARD_SPEED = .8;
    double FORWARD_RAMP_PERCENTAGE = .1;
    double FORWARD_CORRECTION_GAIN = 0.01;

    //TURNING MOVEMENT CONTROLS
    double Kp = 0.01; // Proportional gain
    double Kd = 0.005; // Derivative gain
    double TURN_SPEED_MIN = 0.2;
    double TURN_SPEED_MAX = 0.8;
    double ROTATE_ERROR_DEGREES =3;

    private ElapsedTime runtime = new ElapsedTime();
    public RobotPositionTracker positionTracker;

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
        extensionArmMotor.setPower(0);
        extensionArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Initialize wheel servos
        leftWheelServo = hardwareMap.get(Servo.class, "servo_one");
        rightWheelServo = hardwareMap.get(Servo.class, "servo_two");
        leftWheelServo.setPosition(0.5); // Neutral position
        rightWheelServo.setPosition(0.5); // Neutral position

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize((new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.LEFT))));

        positionTracker = new RobotPositionTracker(hardwareMap.get(GoBildaPinpointDriver.class,"odo"),hardwareMap.get(IMU.class, "imu"));
        telemetry.addData(">", "Charlie 3 is READY for auto mode.  Press START.");
        waitForStart();
    }



    //drive the robot forward a specific time
    public void driveForward(double targetDistanceCM, int sleepMS) {
        long startTime = System.currentTimeMillis();
        double currentPower = FORWARD_MIN_SPEED;
        double correctionPower = 0;  // Initialize correction power to 0 initially
        double initialHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);// Get the initial heading (yaw) from the IMU before the strafe
        double initialPositionX = positionTracker.getXPositionCM();

        while (opModeIsActive()) {
            positionTracker.updatePosition();
            double distanceTraveledX_CM = positionTracker.getXPositionCM() - initialPositionX;

            if (distanceTraveledX_CM >= targetDistanceCM) {
                break;  // Exit the loop if target distance is reached
            }

            // Calculate the percentage of the distance traveled
            double distancePercentage = distanceTraveledX_CM / targetDistanceCM;
            if (distancePercentage < FORWARD_RAMP_PERCENTAGE) {
                telemetry.addData("RAMPING", "UP");
                // Ramp up phase: gradually increase power from FORWARD_MIN_SPEED to FORWARD_SPEED
                currentPower = FORWARD_MIN_SPEED + (FORWARD_SPEED - FORWARD_MIN_SPEED) * (distancePercentage  /  FORWARD_RAMP_PERCENTAGE);
            }else if (distancePercentage < (1 - FORWARD_RAMP_PERCENTAGE)) {
                telemetry.addData("RAMPING", "FULL");
                // Constant power phase: maintain power at FORWARD_SPEED
                currentPower = FORWARD_SPEED;
            } else {
                telemetry.addData("RAMPING", "DOWN");
                // Ramp down phase: gradually decrease power from current speed to 0
                currentPower = FORWARD_SPEED - (FORWARD_SPEED - FORWARD_MIN_SPEED) * ((distancePercentage - (1 - FORWARD_RAMP_PERCENTAGE)) / FORWARD_RAMP_PERCENTAGE);
            }

            // Ensure currentPower stays within the range [FORWARD_MIN_SPEED, FORWARD_SPEED]
            //currentPower = Math.max(FORWARD_MIN_SPEED, Math.min(currentPower, FORWARD_SPEED));
            currentPower = Range.clip(currentPower, FORWARD_MIN_SPEED, FORWARD_SPEED); // Limit power to safe values

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
            telemetry.update();
        }

        // Stop the motors after the loop is done
        frontleftDrive.setPower(0);
        frontrightDrive.setPower(0);
        backleftDrive.setPower(0);
        backrightDrive.setPower(0);

        sleep(sleepMS);
    }
    public void driveBackward(double targetDistanceCM, int sleepMS) {
        long startTime = System.currentTimeMillis();
        double currentPower = FORWARD_MIN_SPEED;
        double correctionPower = 0;  // Initialize correction power to 0 initially
        double initialHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES); // Get the initial heading (yaw) from the IMU

        double initialPositionX = positionTracker.getXPositionCM();


        while (opModeIsActive()) {
            positionTracker.updatePosition();
            double distanceTraveledX_CM = initialPositionX - positionTracker.getXPositionCM();

            if (distanceTraveledX_CM >= targetDistanceCM) {
                break;  // Exit the loop if target distance is reached  // Exit the loop if target distance is reached
            }

            // Calculate the percentage of the distance traveled
            double distancePercentage = distanceTraveledX_CM / targetDistanceCM;

            // Ramp up phase: gradually increase power from FORWARD_MIN_SPEED to FORWARD_SPEED
            if (distancePercentage < FORWARD_RAMP_PERCENTAGE) {
                telemetry.addData("RAMPING", "UP");
                currentPower = FORWARD_MIN_SPEED + (FORWARD_SPEED - FORWARD_MIN_SPEED) * (distancePercentage / FORWARD_RAMP_PERCENTAGE);
            }
            // Full speed phase: maintain power at FORWARD_SPEED
            else if (distancePercentage < (1 - FORWARD_RAMP_PERCENTAGE)) {
                telemetry.addData("RAMPING", "FULL");
                currentPower = FORWARD_SPEED;
            }
            // Ramp down phase: gradually decrease power from current speed to 0
            else {
                telemetry.addData("RAMPING", "DOWN");
                currentPower = FORWARD_SPEED - (FORWARD_SPEED - FORWARD_MIN_SPEED) * ((distancePercentage - (1 - FORWARD_RAMP_PERCENTAGE)) / FORWARD_RAMP_PERCENTAGE);
            }

            // Ensure currentPower stays within the range [FORWARD_MIN_SPEED, FORWARD_SPEED]
            currentPower = Range.clip(currentPower, FORWARD_MIN_SPEED, FORWARD_SPEED); // Limit power to safe values

            // Calculate correction power based on heading deviation
            double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double adjustmentError = initialHeading - currentHeading;

            // Normalize the adjustment error to range [-180, 180]
            if (adjustmentError > 180) adjustmentError -= 360;
            if (adjustmentError < -180) adjustmentError += 360;

            // Ramp down correction power similarly as the main currentPower
            double correctionRampDownFactor = currentPower / FORWARD_SPEED; // Same ramp factor as currentPower
            correctionPower = FORWARD_CORRECTION_GAIN * adjustmentError * correctionRampDownFactor;

            // Apply the power to the motors (reverse direction for backward movement)
            frontleftDrive.setPower(currentPower - correctionPower);  // Reverse direction for backward motion
            frontrightDrive.setPower(currentPower + correctionPower); // Reverse direction for backward motion
            backleftDrive.setPower(-currentPower - correctionPower); // Reverse direction for backward motion
            backrightDrive.setPower(-currentPower + correctionPower); // Reverse direction for backward motion

            // Telemetry for monitoring
            telemetry.addData("Wheel Power", currentPower);
            telemetry.addData("CORRECTION POWER", correctionPower);
            telemetry.update();
        }

        // Stop the motors after the loop is done
        frontleftDrive.setPower(0);
        frontrightDrive.setPower(0);
        backleftDrive.setPower(0);
        backrightDrive.setPower(0);

        sleep(sleepMS);
    }


    public void strafeLeft(double targetDistanceCM, int sleepMS) {
        long startTime = System.currentTimeMillis();
        double currentPower = STRAFE_MIN_SPEED;
        double correctionPower = 0;  // Initialize correction power to 0 initially
        runtime.reset();

        // Get the initial heading (yaw) from the IMU before the strafe
        double initialHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double initialPositionY = positionTracker.getXPositionCM();

        while (opModeIsActive()) {
            double elapsedTime = runtime.milliseconds();
            // Calculate current encoder position for the X (side) wheel and the distance traveled
            positionTracker.updatePosition();
            double distanceTraveledY_CM = initialPositionY - positionTracker.getYPositionCM();

            // Check if the robot has traveled the desired distance
            if (distanceTraveledY_CM >= targetDistanceCM) {
                break;  // Exit the loop if target distance is reached  // Exit the loop if target distance is reached
            }
            // Calculate the percentage of the distance traveled for ramping
            double distancePercentage = distanceTraveledY_CM / targetDistanceCM;

            // Ramp up phase: gradually increase power from STRAFE_MIN_SPEED to STRAFE_SPEED
            if (distancePercentage < STRAFE_RAMP_PERCENTAGE) {
                telemetry.addData("RAMPING", "UP");
                currentPower = STRAFE_MIN_SPEED + (STRAFE_SPEED - STRAFE_MIN_SPEED) * (distancePercentage / STRAFE_RAMP_PERCENTAGE);
            }
            // Constant power phase: maintain power at STRAFE_SPEED
            else if (distancePercentage < (1 - STRAFE_RAMP_PERCENTAGE)) {
                telemetry.addData("RAMPING", "FULL");
                currentPower = STRAFE_SPEED;
            }
            // Ramp down phase: gradually decrease power from STRAFE_SPEED to 0
            else {
                telemetry.addData("RAMPING", "DOWN");
                currentPower = STRAFE_SPEED - (STRAFE_SPEED - STRAFE_MIN_SPEED) * ((distancePercentage - (1 - STRAFE_RAMP_PERCENTAGE)) / STRAFE_RAMP_PERCENTAGE);
            }

            // Ensure currentPower stays within the range [0, STRAFE_SPEED]
            currentPower = Range.clip(currentPower, STRAFE_MIN_SPEED, STRAFE_SPEED); // Limit power to safe values

            // Calculate correction power based on heading deviation
            double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double adjustmentError = initialHeading - currentHeading;

            // Normalize the adjustment error to range [-180, 180]
            if (adjustmentError > 180) adjustmentError -= 360;
            if (adjustmentError < -180) adjustmentError += 360;

            // Ramp down correction power similarly as the main currentPower
            double correctionRampDownFactor = currentPower / STRAFE_SPEED; // Same ramp factor as currentPower
            correctionPower = STRAFE_CORRECTION_GAIN * adjustmentError * correctionRampDownFactor;

            // Apply power to motors for strafing with correction
            frontleftDrive.setPower(currentPower + correctionPower);  // Front left motor forward with correction
            frontrightDrive.setPower(-currentPower - correctionPower); // Front right motor backward with correction
            backleftDrive.setPower(-currentPower + correctionPower);   // Left back motor backward with correction
            backrightDrive.setPower(currentPower - correctionPower);   // Right back motor forward with correction

            // Telemetry for monitoring
            telemetry.addData("CURRENT POWER", currentPower);
            telemetry.addData("CORRECTION POWER", correctionPower);
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


    public void strafeRight(double targetDistanceCM, int sleepMS) {
        long startTime = System.currentTimeMillis();
        double currentPower = STRAFE_MIN_SPEED;
        double correctionPower = 0;  // Initialize correction power to 0 initially
        runtime.reset();

        // Get the initial heading (yaw) from the IMU before the strafe
        double initialHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        // Track initial encoder position for the Y (side) odometry wheel
        double initialPositionY = positionTracker.getYPositionCM();
        while (opModeIsActive()) {
            double elapsedTime = runtime.milliseconds();
            // Calculate current encoder position for the X (side) wheel and the distance traveled
            double distanceTraveledY_CM = initialPositionY - positionTracker.getYPositionCM();

            // Check if the robot has traveled the desired distance
            if (distanceTraveledY_CM >= targetDistanceCM) {
                break;  // Exit the loop if target distance is reached  // Exit the loop if target distance is reached
            }

            // Calculate the percentage of the distance traveled for ramping
            double distancePercentage = distanceTraveledY_CM / targetDistanceCM;

            // Ramp up phase: gradually increase power from STRAFE_MIN_SPEED to STRAFE_SPEED
            if (distancePercentage < STRAFE_RAMP_PERCENTAGE) {
                telemetry.addData("RAMPING", "UP");
                currentPower = STRAFE_MIN_SPEED + (STRAFE_SPEED - STRAFE_MIN_SPEED) * (distancePercentage / STRAFE_RAMP_PERCENTAGE);
            }
            // Constant power phase: maintain power at STRAFE_SPEED
            else if (distancePercentage < (1 - STRAFE_RAMP_PERCENTAGE)) {
                telemetry.addData("RAMPING", "FULL");
                currentPower = STRAFE_SPEED;
            }
            // Ramp down phase: gradually decrease power from STRAFE_SPEED to 0
            else {
                telemetry.addData("RAMPING", "DOWN");
                currentPower = STRAFE_SPEED - (STRAFE_SPEED - STRAFE_MIN_SPEED) * ((distancePercentage - (1 - STRAFE_RAMP_PERCENTAGE)) / STRAFE_RAMP_PERCENTAGE);
            }

            // Ensure currentPower stays within the range [0, STRAFE_SPEED]
            currentPower = Range.clip(currentPower, STRAFE_MIN_SPEED, STRAFE_SPEED); // Limit power to safe values

            // Calculate correction power based on heading deviation
            double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double adjustmentError = initialHeading - currentHeading;

            // Normalize the adjustment error to range [-180, 180]
            if (adjustmentError > 180) adjustmentError -= 360;
            if (adjustmentError < -180) adjustmentError += 360;

            // Ramp down correction power similarly as the main currentPower
            double correctionRampDownFactor = currentPower / STRAFE_SPEED; // Same ramp factor as currentPower
            correctionPower = STRAFE_CORRECTION_GAIN * adjustmentError * correctionRampDownFactor;

            // Apply power to motors for strafing with correction
            frontleftDrive.setPower(-currentPower + correctionPower);  // Front left motor backward with correction
            frontrightDrive.setPower(currentPower - correctionPower); // Front right motor forward with correction
            backleftDrive.setPower(currentPower + correctionPower);   // Left back motor forward with correction
            backrightDrive.setPower(-currentPower - correctionPower);   // Right back motor backward with correction

            // Telemetry for monitoring
            telemetry.addData("CURRENT POWER", currentPower);
            telemetry.addData("CORRECTION POWER", correctionPower);
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
/*

    public void moveToPosition(double targetX, double targetY, int sleepMS) {
        // Get current position using odometry

        double currentX = positionTracker.getXPositionCM();  // Assume this function gets the current X position
        double currentY = positionTracker.getYPositionCM(); // Assume this function gets the current Y position

        // Calculate the difference in X and Y
        double deltaX = targetX - currentX;
        double deltaY = targetY - currentY;

        // Move along the X axis (strafe left or right)
        if (Math.abs(deltaX) > 1) { // Only move if the difference is significant
            if (deltaX > 0) {
                // Strafe right to reach the target X position
                strafeRight(Math.abs(deltaX), sleepMS);
            } else {
                // Strafe left to reach the target X position
                strafeLeft(Math.abs(deltaX), sleepMS);
            }
        }

        // Move along the Y axis (drive forward or backward)
        if (Math.abs(deltaY) > 1) { // Only move if the difference is significant
            if (deltaY > 0) {
                // Drive forward to reach the target Y position
                driveForward(Math.abs(deltaY), sleepMS);
            } else {
                // Drive backward to reach the target Y position
                driveBackward(Math.abs(deltaY), sleepMS);
            }
        }

        // Optionally, add final adjustments if necessary
        telemetry.addData("Target Position", "X: " + targetX + " Y: " + targetY);
        telemetry.addData("Current Position", "X: " + currentX + " Y: " + currentY);
        telemetry.update();

        // Optional sleep to allow the robot to settle
        sleep(sleepMS);
    }
*/


    //turn robot left a specific degrees (from current position)
    public void turnLeft(double targetAngle, int sleepMS) {
        sleep(200);
        runtime.reset();

        // Initial heading from IMU
        //double zeroAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        positionTracker.updatePosition();
        double startHeading = positionTracker.currentHeading;
        double currentAngle;
        double currentError;
        double lastError = 0;
        double derivative;
        double motorPower;

        while (opModeIsActive()) {
            // Get the current angle from the IMU
            positionTracker.updatePosition();

            //currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            // Calculate the error between target and current angle
            currentError = targetAngle - positionTracker.currentHeading;

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
            telemetry.addData("Current Angle", positionTracker.currentHeading);
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


    public void turnRight(double targetAngle, int sleepMS) {
        sleep(200);
        runtime.reset();

        positionTracker.updatePosition();
        double startAngle = positionTracker.currentHeading;

        double targetAbsoluteAngle = startAngle - targetAngle; // Subtract the targetAngle for a right turn
        double currentError;
        double lastError = 0;
        double derivative;
        double motorPower;

        while (opModeIsActive()) {
            positionTracker.updatePosition();
            // Calculate the error between target and current angle
            currentError = targetAbsoluteAngle - positionTracker.currentHeading;
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
            frontrightDrive.setPower(-motorPower); // Positive power for turning right
            frontleftDrive.setPower(motorPower); // Negative power for turning right
            backleftDrive.setPower(motorPower);  // Negative power for turning right
            backrightDrive.setPower(-motorPower);  // Positive power for turning right

            // Log telemetry for debugging
            telemetry.addData("Target Absolute Angle", targetAbsoluteAngle);
            telemetry.addData("Current Angle", positionTracker.currentHeading);
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
    public void moveArmEncoder(int targetPosition, int sleepMS) {
        long actionStartTime;
        int startArmPosition = armMotor.getCurrentPosition();
        double currentArmSpeed = 0;

        // If the arm is already at or above the target height
        // Step 1: Move the arm to the target height

        armTargetPosition = Range.clip(targetPosition, ARM_MIN_POSITION, ARM_MAX_POSITION); // Limit power to safe values

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
                float rampFactor = Math.min(1.0f, Math.max(0.0f, (float) distanceFromStart / ARM_RAMP_TICKS));
                currentArmSpeed = ARM_MIN_SPEED + (ARM_MAX_SPEED - ARM_MIN_SPEED) * rampFactor;
            }
            else if (distanceFromStart > ARM_RAMP_TICKS && distanceToTarget > ARM_RAMP_TICKS) {
                // State 2: Full Speed (between the first and last 50 ticks)
                currentArmSpeed = ARM_MAX_SPEED;
            }
            else if (distanceToTarget <= ARM_RAMP_TICKS) {
                // State 3: Ramp-Down (when within the last XXX ticks of movement)
                float rampFactor = Math.min(1.0f, Math.max(0.0f, (float) distanceToTarget / ARM_RAMP_TICKS));
                // Interpolate speed using the ramp factor
                currentArmSpeed = ARM_MIN_SPEED + (ARM_MAX_SPEED - ARM_MIN_SPEED) * rampFactor;
            }
            currentArmSpeed = Range.clip(currentArmSpeed, ARM_MIN_SPEED, ARM_MAX_SPEED); // Limit power to safe values
            armMotor.setPower(currentArmSpeed);
            addTelemetry();

            if (distanceToTarget < MOTOR_TOLERANCE) {
                break;
            }
        }

        // Stop motors at the end to ensure no lingering movement
        armMotor.setPower(ARM_MIN_SPEED);
        extensionArmMotor.setPower(0);
        sleep(sleepMS);
    }

    public void moveExtensionEncoder(int targetPosition, int sleepMS) {
        long actionStartTime;
        double currentExtensionSpeed = 0;
        int startExtensionPosition = extensionArmMotor.getCurrentPosition();
        extensionTargetPosition = Range.clip(targetPosition, EXTENSION_MIN_POSITION, EXTENSION_MAX_POSITION); // Limit power to safe values
        extensionArmMotor.setTargetPosition(extensionTargetPosition);
        extensionArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        currentExtensionSpeed = EXTENSION_MIN_SPEED;
        extensionArmMotor.setPower(currentExtensionSpeed);
        actionStartTime = System.currentTimeMillis();
        while (extensionArmMotor.isBusy() ) {

            int totalTicks = Math.abs(targetPosition - extensionArmMotor.getCurrentPosition()); // Total ticks for the move
            while (opModeIsActive() && extensionArmMotor.isBusy()) {
                int currentPos = armMotor.getCurrentPosition();
                int distanceToTarget = Math.abs(targetPosition - currentPos);
                int distanceFromStart = Math.abs(currentPos - startExtensionPosition);

                // Calculate the arm power based on ramping up or down
                if (distanceFromStart < EXTENSION_RAMP_TICKS) {
                    // State 1: Ramp-Up (when within the first XXX ticks of movement)
                    float rampFactor = Math.min(1.0f, Math.max(0.0f, (float) distanceFromStart / EXTENSION_RAMP_TICKS));
                    currentExtensionSpeed = EXTENSION_MIN_SPEED + (EXTENSION_MAX_SPEED - EXTENSION_MIN_SPEED) * rampFactor;
                }
                else if (distanceFromStart > EXTENSION_RAMP_TICKS && distanceToTarget > EXTENSION_RAMP_TICKS) {
                    // State 2: Full Speed (between the first and last 50 ticks)
                    currentExtensionSpeed = EXTENSION_MAX_SPEED;
                }
                else if (distanceToTarget <= EXTENSION_RAMP_TICKS) {
                    // State 3: Ramp-Down (when within the last XXX ticks of movement)
                    float rampFactor = Math.min(1.0f, Math.max(0.0f, (float) distanceToTarget / EXTENSION_RAMP_TICKS));
                    // Interpolate speed using the ramp factor
                    currentExtensionSpeed = EXTENSION_MIN_SPEED + (EXTENSION_MAX_SPEED - EXTENSION_MIN_SPEED) * rampFactor;

                }
                currentExtensionSpeed = Range.clip(currentExtensionSpeed, EXTENSION_MIN_SPEED, EXTENSION_MAX_SPEED); // Limit power to safe values
                extensionArmMotor.setPower(currentExtensionSpeed);
                addTelemetry();

                if (distanceToTarget < MOTOR_TOLERANCE) {
                    break;
                }
            }
        }
        sleep(sleepMS);
    }

    public void outputClaw(int runMS, int sleepMS) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        leftWheelServo.setPosition(SERVO_BACKWARD);  // Spin inward
        rightWheelServo.setPosition(SERVO_FORWARD); // Spin inward
        // Wait for the specified duration
        while (opModeIsActive() && timer.milliseconds() < runMS) {
            telemetry.addData("Output Claw", "Running for %d ms", runMS);
            telemetry.update();
        }
        leftWheelServo.setPosition(SERVO_STOPPED);  // Neutral
        rightWheelServo.setPosition(SERVO_STOPPED); // Neutral
        sleep(sleepMS);
    }

    public void intakeClaw(int runMS, int sleepMS) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        leftWheelServo.setPosition(SERVO_FORWARD);  // Spin inward
        rightWheelServo.setPosition(SERVO_BACKWARD); // Spin inward
        // Wait for the specified duration
        while (opModeIsActive() && timer.milliseconds() < runMS) {
            telemetry.addData("Intake Claw", "Running for %d ms", runMS);
            telemetry.update();
        }
        leftWheelServo.setPosition(SERVO_STOPPED);  // Neutral
        rightWheelServo.setPosition(SERVO_STOPPED); // Neutral
        sleep(sleepMS);
    }

    //shut down the robot (end of auto)
    public void closeRobot() {
        moveExtensionEncoder(0,0);
        moveArmEncoder(0,0);
    }

    //get the current angle of the robot
    public double getHeading() {
        double angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        return angle;
    }


    public void rotateToHeading(double targetAngle, int sleepMS) {
        sleep(200); // Optional small delay before starting
        runtime.reset();

        // Initial heading from IMU
        double currentAngle;
        double currentError;
        double lastError = 0;
        double derivative;
        double motorPower;

        while (opModeIsActive()) {
            currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            currentError = targetAngle - currentAngle;

            // Normalize the error to the range [-180, 180]
            currentError = (currentError + 360) % 360; // Ensure positive range
            if (currentError > 180) currentError -= 360; // Wrap to [-180, 180]
            derivative = currentError - lastError;
            motorPower = (Kp * currentError) + (Kd * derivative);
            motorPower = Range.clip(motorPower, -TURN_SPEED_MAX, TURN_SPEED_MAX);
            if (Math.abs(motorPower) < TURN_SPEED_MIN && Math.abs(currentError) > ROTATE_ERROR_DEGREES) {
                motorPower = Math.copySign(TURN_SPEED_MIN, motorPower);
            }

            frontrightDrive.setPower(-motorPower);
            frontleftDrive.setPower(motorPower);
            backleftDrive.setPower(motorPower);
            backrightDrive.setPower(-motorPower);

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
