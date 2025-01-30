
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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

    double DRIVING_SLOW =0.7;
    double DRIVING_SLOW_AUTO = 0.7;

    // Arm motor limits and power
    int ARM_MIN_POSITION =100;    // Minimum encoder position (fully Lowered// )
    int ARM_MAX_POSITION = 1300; // Maximum encoder position (fully Raised)
    double ARM_BASE_POWER = 0.2;
    double ARM_EXTRA_FORCE = 0.01;
    double ARM_MAX_SPEED = 0.9;
    double ARM_MIN_SPEED = 0.3;
    double ARM_RAMP_TICKS = 50;
    double ARM_MANUAL_MAX_SPEED = 40;

    // Extension limits and power
    int EXTENSION_MIN_POSITION = 0;    // Minimum height (fully lowered)
    int EXTENSION_MAX_POSITION = 2200; // Maximum height (fully raised)
    double EXTENSION_BASE_POWER = 0.3;
    double EXTENSION_EXTRA_FORCE = 0.6;

    double EXTENSION_MIN_SPEED = 0.3; // How far to move per iterationfsampleRelease
    double EXTENSION_MAX_SPEED = 0.9; // How far to move per iteration
    double EXTENSION_RAMP_TICKS = 50;
    double EXTENSION_MANUAL_MAX_SPEED = 100;

    //tolernances
    int MOTOR_TOLERANCE = 5; // Acceptable error in encoder ticks
    double POSITION_TOLERANCE_CM = 1;
    double HEADING_TOLERANCE_DEGREES = 2;

    //servo limits
    double SERVO_STOPPED = 0.5;
    double SERVO_FORWARD = 1;
    double SERVO_BACKWARD = 0;

    //drive speeds
    double DRIVE_MAX_POWER = 0.7; // Maximum power
    double DRIVE_MIN_POWER = 0.15; // Minimum power to prevent stalling
    double DRIVE_SLOW_THRESHOLD = 10.0; // Distance (CM) where slowdown begins
    double DRIVE_CRAWL_THRESHOLD = 2.0; // Distance (CM) where slow crawl is enforced

    //turn speeds
    double TURN_MAX_POWER = 0.6; // Maximum turning power
    double TURN_MIN_POWER = 0.15; // Minimum turning power for precision
    double TURN_SLOWDOWN_THRESHOLD = Math.toRadians(20.0); // Angle threshold for starting to slow down

    //pre-defined positions
    int DRIVE_ARM_POSITION = 200;

    int HOOK_EXTENSION_POSITION = 1651;
    int HOOK_ARM_HEIGHT = 761;
    int HOOK_DEGREES = 0;
    int HOOK_POS_X = 58;
    int HOOK_POS_Y = 158;
    int HOOK_ARM_HEIGHT_2 = 735;
    int POST_HOOK_POS_X = 40;
    int POST_HOOK_POS_Y = 158;

    int PICKUP_SAMPLE_ARM_HEIGHT = 286;
    int PICKUP_SAMPLE_EXTENSION_POSITION = 1630;
    int PICKUP_SAMPLE_DEGREES = 180;
    int PICKUP_SAMPLE_POS_X = 45;
    int PICKUP_SAMPLE_POS_Y = 37;
    int PICKUP_SAMPLE_POS_INTAKE_X = 35;
    int PICKUP_SAMPLE_POS_NOPICKUP_X = 70;

    int RELEASE_SAMPLE_ARM_HEIGHT = 760;
    int RELEASE_SAMPLE_EXTENSION_POSITION = 1840;
    int RELEASE_SAMPLE_DEGREES = 133;
    int RELEASE_SAMPLE_POS_X = 35;
    int RELEASE_SAMPLE_POS_Y = 284;

    int PUSH_FIRST_BLOCK_POS_X_1 = 521;
    int PUSH_FIRST_BLOCK_POS_Y_1 = 69;
    int PUSH_FIRST_BLOCK_POS_X_2 = 132;
    int PUSH_FIRST_BLOCK_POS_Y_2 = 69;
    int PUSH_FIRST_BLOCK_POS_X_3 = 132;
    int PUSH_FIRST_BLOCK_POS_Y_3 = 47;
    int PUSH_FIRST_BLOCK_POS_X_4 = 8;
    int PUSH_FIRST_BLOCK_POS_Y_4 = 47;

    int PUSH_SECOND_BLOCK_POS_X_1 = 132;
    int PUSH_SECOND_BLOCK_POS_Y_1 = 47;
    int PUSH_SECOND_BLOCK_POS_X_2 = 132;
    int PUSH_SECOND_BLOCK_POS_Y_2 = 22;
    int PUSH_SECOND_BLOCK_POS_X_3 = 20;
    int PUSH_SECOND_BLOCK_POS_Y_3 = 17;

    int PICKUP_BLOCK_POS_X = 132;//ESTIMATE ONLY
    int PICKUP_BLOCK_POS_Y = 47;//ESTIMATE ONLY

    int PARK_LEFT_AUTO_POS_1_X = 100;//ESTIMATE ONLY
    int PARK_LEFT_AUTO_POS_1_Y = 250;//ESTIMATE ONLY

    int PARK_LEFT_AUTO_POS_2_X = 200;//ESTIMATE ONLY
    int PARK_LEFT_AUTO_POS_2_Y = 200;//ESTIMATE ONLY

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
    public pickupSampleGroundState samplePickupState = pickupSampleGroundState.IDLE;
    public releaseSampleFirstBucketState sampleReleaseState = releaseSampleFirstBucketState.IDLE;

    public manualServoState tmpServoState = manualServoState.IDLE;
    public driveToPositionState tmpDriveState = driveToPositionState.IDLE;
    public wristServoState tempServoState = wristServoState.IDLE;

    int tmpArmPositionHolder = 0;
    int tmpExtensionPositionHolder = 0;
    long tmpActionStartTime = 0;
    public RobotPositionTracker positionTracker;

    public void init(HardwareMap hwMap, Telemetry telem) {
        hardwareMap = hwMap;
        telemetry = telem;
        // Define and Initialize wheel Motors
        frontleftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontrightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backrightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        backleftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
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
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //armTargetPosition = ARM_MIN_POSITION;

        // Initialize extension arm motor
        extensionArmMotor = hardwareMap.get(DcMotor.class, "arm_extendo");
        extensionArmMotor.setDirection(DcMotor.Direction.REVERSE);
        extensionArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize wheel servos
        leftWheelServo = hardwareMap.get(Servo.class, "servo_one");
        rightWheelServo = hardwareMap.get(Servo.class, "servo_two");
        Wrist = hardwareMap.get(Servo.class, "wrist");

        leftWheelServo.setPosition(0.5); // Neutral position
        rightWheelServo.setPosition(0.5); // Neutral position
        Wrist.setPosition(0.5);

        touchsensor = hardwareMap.get(TouchSensor.class, "touch sensor");
        imu = hardwareMap.get(IMU.class,"imu");
        imu.initialize((new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.LEFT))));

        positionTracker = new RobotPositionTracker(hardwareMap.get(GoBildaPinpointDriver.class,"odo"),hardwareMap.get(IMU.class, "imu"));
    }

//  Telemetry
    public void addTelemetry() {
        telemetry.addData("POSITION - Current X",  "%.2f", positionTracker.getXPositionCM());
        telemetry.addData("POSITION - Current Y",  "%.2f", positionTracker.getYPositionCM());
        telemetry.addData("POSITION - Current deg", positionTracker.getHeadingDegrees());
        //telemetry.addData("Button pressed", touchsensor.isPressed());
//        telemetry.addData("front",  "%.2f", currentForward);
//        telemetry.addData("turn", "%.2f", currentTurn);
//        telemetry.addData("strafe", "%.2f", currentStrafe);

        telemetry.addData("EXTENSION - Current Position", extensionArmMotor.getCurrentPosition());
        telemetry.addData("EXTENSION - Target Position", extensionTargetPosition);
        //telemetry.addData("Extension Calculated Power", currentExtensionPower);
        //telemetry.addData("Extension Motor Busy", extensionArmMotor.isBusy());

        telemetry.addData("ARM - Current Position", armMotor.getCurrentPosition());
        telemetry.addData("ARM - Target Position", armTargetPosition);
        //telemetry.addData("Arm Calculated Min Position", dynamicArmMinPosition);
        //telemetry.addData("Arm Calculated Power", currentArmPower);

        //telemetry.addData("Left Servo Position", leftWheelServo.getPosition());
        //telemetry.addData("Right Servo Position", rightWheelServo.getPosition());*/


        //telemetry.update();
    }

    public void updateTelemetry() {
        telemetry.update();
    }

    public void driveWheels(double tmpForward, double tmpTurn, double tmpStrafe, boolean human) {
        currentForward = tmpForward;
        currentTurn = tmpTurn;
        currentStrafe = tmpStrafe;

        if (human) {
            // Apply dead zone and smoothing
            currentForward = Math.abs(currentForward) > 0.05 ? Math.pow(currentForward, 3) : 0.0;
            //currentTurn = Math.abs(currentTurn) > 0.05 ? Math.pow(currentTurn, 3) : 0.0;
            if (Math.abs(currentTurn) > 0.05) {
                currentTurn = currentTurn; // Keep it linear
            } else {
                currentTurn = 0.0;
            }
            currentStrafe = Math.abs(currentStrafe) > 0.05 ? Math.pow(currentStrafe, 3) : 0.0;
        }


        // Combine inputs for omnidirectional control
        double frontLeftPower = -currentForward + -currentTurn + -currentStrafe;
        double frontRightPower = -currentForward - -currentTurn - -currentStrafe;
        double backLeftPower = currentForward + -currentTurn - -currentStrafe;
        double backRightPower = currentForward - -currentTurn + -currentStrafe;

        // Normalize power values to avoid exceeding 1.0
        double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
        if (human) {
            frontLeftPower = frontLeftPower * DRIVING_SLOW;
            frontRightPower = frontRightPower * DRIVING_SLOW;
            backLeftPower = backLeftPower * DRIVING_SLOW;
            backRightPower = backRightPower * DRIVING_SLOW;
        }
        if (!human && currentTurn == 0) {//do not slow down turns - only movement.
            frontLeftPower = frontLeftPower * DRIVING_SLOW_AUTO;
            frontRightPower = frontRightPower * DRIVING_SLOW_AUTO;
            backLeftPower = backLeftPower * DRIVING_SLOW_AUTO;
            backRightPower = backRightPower * DRIVING_SLOW_AUTO;
        }
        // If the maxPower exceeds 1.0, normalize all power values by dividing by maxPower
        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        // Set motor powers
        frontleftDrive.setPower(frontLeftPower);
        frontrightDrive.setPower(frontRightPower);
        backleftDrive.setPower(backLeftPower);
        backrightDrive.setPower(backRightPower);
        telemetry.addData("DRIVE POWER - forward", currentForward);
        telemetry.addData("DRIVE POWER - Turn", currentTurn);
        telemetry.addData("DRIVE POWER - Strafe", currentStrafe);

        telemetry.addData("WHEEL POWER - frontLeft", frontLeftPower);
        telemetry.addData("WHEEL POWER - frontRight", frontRightPower);
        telemetry.addData("WHEEL POWER - backLeft", backLeftPower);
        telemetry.addData("WHEEL POWER - backRight", backRightPower);
        long currentTime = System.currentTimeMillis();
        positionTracker.updatePosition();
    }

    public boolean driveToPosition(double targetXCM, double targetYCM, double targetHeadingDegrees) {
        if (tmpDriveState == driveToPositionState.IDLE) {
            tmpDriveState = driveToPositionState.DRIVE;
            //tmpDriveState = driveToPositionState.TURN;
        }
        // Convert targetHeading from degrees to radians
        double targetHeadingRadians = Math.toRadians(targetHeadingDegrees);

        // Update the robot's current position and heading
        positionTracker.updatePosition();
        double currentX = positionTracker.getXPositionCM();
        double currentY = positionTracker.getYPositionCM();
        double currentHeading = positionTracker.getHeading();

        // Calculate differences to target
        double deltaX = targetXCM - currentX;
        double deltaY = targetYCM - currentY;
        double distanceToTarget = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        double deltaHeading = targetHeadingRadians - currentHeading;

        // Normalize deltaHeading to the range [-π, π] to ensure shortest turn
        if (deltaHeading > Math.PI) deltaHeading -= 2 * Math.PI;
        if (deltaHeading < -Math.PI) deltaHeading += 2 * Math.PI;

        if (Math.abs(deltaHeading) < Math.toRadians(HEADING_TOLERANCE_DEGREES)) {
            deltaHeading = 0; // Snap to 0 to avoid unnecessary small rotations
        }

        telemetry.addData("DRIVE STATE", tmpDriveState);
        telemetry.addData("TARGET POS - X", targetXCM);
        telemetry.addData("TARGET POS - Y", targetYCM);
        telemetry.addData("DELTA POS - X", deltaX);
        telemetry.addData("DELTA POS - Y", deltaY);

        //telemetry.addData("Current Heading", Math.toDegrees(currentHeading));
        telemetry.addData("TARGET HEADING", targetHeadingDegrees);
        telemetry.addData("TARGET HEADING DELTA", Math.toDegrees(deltaHeading));


        // Step 1: Move to the target position
        if (distanceToTarget > POSITION_TOLERANCE_CM && tmpDriveState == driveToPositionState.DRIVE) {
            // Calculate the angle to the target relative to the robot's position
            double angleToTarget = Math.atan2(deltaY, deltaX);
            double relativeAngleToTarget = angleToTarget - currentHeading;

            // Normalize the relative angle to the range [-π, π]
            if (relativeAngleToTarget > Math.PI) relativeAngleToTarget -= 2 * Math.PI;
            if (relativeAngleToTarget < -Math.PI) relativeAngleToTarget += 2 * Math.PI;

            //double drivePower = calculateDrivePower(distanceToTarget);

            // Calculate forward and strafe powers based on the relative angle
            double forwardPower = Math.cos(relativeAngleToTarget) * distanceToTarget;
            double strafePower = Math.sin(relativeAngleToTarget) * distanceToTarget;
            //double forwardPower = Math.cos(relativeAngleToTarget) * drivePower;
            //double strafePower = Math.sin(relativeAngleToTarget) * drivePower;


            // Normalize power values to prevent exceeding max power
            double maxPower = Math.max(Math.abs(forwardPower), Math.abs(strafePower));
            if (maxPower > 1.0) {
                forwardPower /= maxPower;
                strafePower /= maxPower;
            }

            // Send adjusted power to the drive system
            driveWheels(forwardPower, 0, -strafePower, false);

            //telemetry.addData("Forward Power", forwardPower);
            //telemetry.addData("Strafe Power", -strafePower);
        } else {
            if (tmpDriveState == driveToPositionState.DRIVE) {

                tmpDriveState = driveToPositionState.TURN;
                //tmpDriveState = driveToPositionState.COMPLETE;
                driveWheels(0, 0, 0, false); // Stop movement
            }
        }

        // Step 2: Adjust heading if needed
        if (Math.abs(deltaHeading) > Math.toRadians(HEADING_TOLERANCE_DEGREES) && tmpDriveState == driveToPositionState.TURN) {
            double turnPower = calculateTurnPower(deltaHeading);

            // Ensure turnPower is applied in the correct direction
            turnPower *= Math.signum(deltaHeading);

            // Apply turn power
            driveWheels(0, -turnPower, 0, false);

            //telemetry.addData("Turn Power", -turnPower);
        } else {
            if (tmpDriveState == driveToPositionState.TURN) {
                driveWheels(0, 0, 0, false);
                tmpDriveState = driveToPositionState.ADJUST;
                //tmpDriveState = driveToPositionState.COMPLETE;
                //tmpDriveState = driveToPositionState.DRIVE;
            }
        }


        // Step 3: Adjust position if needed (after a turn)
        if (distanceToTarget > POSITION_TOLERANCE_CM && tmpDriveState == driveToPositionState.ADJUST) {
            // Calculate the angle to the target relative to the robot's position
            double angleToTarget = Math.atan2(deltaY, deltaX);
            double relativeAngleToTarget = angleToTarget - currentHeading;

            // Normalize the relative angle to the range [-π, π]
            if (relativeAngleToTarget > Math.PI) relativeAngleToTarget -= 2 * Math.PI;
            if (relativeAngleToTarget < -Math.PI) relativeAngleToTarget += 2 * Math.PI;

            //double drivePower = calculateDrivePower(distanceToTarget);
            // Calculate forward and strafe powers based on the relative angle
            double forwardPower = Math.cos(relativeAngleToTarget) * distanceToTarget;
            double strafePower = Math.sin(relativeAngleToTarget) * distanceToTarget;
            //double forwardPower = Math.cos(relativeAngleToTarget) * drivePower;
            //double strafePower = Math.sin(relativeAngleToTarget) * drivePower;

            // Normalize power values to prevent exceeding max power
            double maxPower = Math.max(Math.abs(forwardPower), Math.abs(strafePower));
            if (maxPower > 1.0) {
                forwardPower /= maxPower;
                strafePower /= maxPower;
            }

            // Send adjusted power to the drive system
            driveWheels(forwardPower, 0, -strafePower, false);

        } else {
            if (tmpDriveState == driveToPositionState.ADJUST) {
                tmpDriveState = driveToPositionState.COMPLETE;
                driveWheels(0, 0, 0, false); // Stop movement
            }
        }

        return tmpDriveState == driveToPositionState.COMPLETE;
    }

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

    private double calculateTurnPower(double deltaHeading) {
        // If within the final tolerance range, stop turning
        if (Math.abs(deltaHeading) <= Math.toRadians(HEADING_TOLERANCE_DEGREES)) {
            telemetry.addData("TURN ","STOP");
            return 0.0; // Stop turning when within tolerance range
        }

        // If the robot is within the slow down threshold, apply the slow turn power
        if (Math.abs(deltaHeading) <= TURN_SLOWDOWN_THRESHOLD) {
            telemetry.addData("TURN ","SLOW");
            return TURN_MIN_POWER;// * Math.signum(deltaHeading); // Slow down the turn
        }
        telemetry.addData("TURN ","FULL");
        // Apply full turn power for larger heading differences
        return TURN_MAX_POWER;//maxTurnPower * Math.signum(deltaHeading); // Apply max turn power
    }


    public void moveArm( float leftStickY) {
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
            if (extensionPosition == EXTENSION_MAX_POSITION) {
                dynamicArmMinPosition = 200;
            } else if (extensionPosition >= 900) {
                // Linearly interpolate between 900 and 2200 for the range [100, 200]
                //this code may not work correctly
                dynamicArmMinPosition = (int) (100 + (extensionPosition - 900) * (200 - 100) / (EXTENSION_MAX_POSITION - 900));
            } else if (extensionPosition < 900 && extensionPosition > EXTENSION_MIN_POSITION) {
                dynamicArmMinPosition = ARM_MIN_POSITION;
            } else {
                dynamicArmMinPosition = 0;
            }

            // Clamp the target position to ensure the arm doesn't exceed the boundaries
            armTargetPosition = Range.clip(armTargetPosition, dynamicArmMinPosition, ARM_MAX_POSITION);
            // Set the arm motor's target position
            armMotor.setTargetPosition(armTargetPosition);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            currentArmPower = calcArmPower();
            // Set motor power based on your control system logic
            armMotor.setPower(currentArmPower);
        }
        else {
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
        }
        else {
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
        }
        else {
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
        }
        else {
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

    public void moveIntake(boolean inward,boolean outward) {
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
        }else {
            if (tmpServoState == manualServoState.INPUT || tmpServoState == manualServoState.OUTPUT) {
                tmpServoState = manualServoState.IDLE;
                leftWheelServo.setPosition(SERVO_STOPPED);  // Neutral
                rightWheelServo.setPosition(SERVO_STOPPED); // Neutral
            }
        }
        // --- END WHEEL SERVO CONTROL ---
    }

    public boolean moveIntakeTimed(boolean inward,boolean outward, int milliseconds, LinearOpMode opMode) {
        // Create an instance of ElapsedTime to track the duration
        ElapsedTime timer = new ElapsedTime();
        boolean complete = false;
        // --- WHEEL SERVO CONTROL ---
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

            // Reset and start the timer
            timer.reset();

            // Continue running until the specified duration has elapsed
            while (opMode.opModeIsActive() && timer.milliseconds() < milliseconds) {
                // Optionally, include idle() or sleep() to prevent CPU overuse
                // idle();
            }

            // Time has elapsed; stop the servos
            tmpServoState = manualServoState.IDLE;
            leftWheelServo.setPosition(SERVO_STOPPED);  // Neutral
            rightWheelServo.setPosition(SERVO_STOPPED); // Neutral
            complete = true;
        }
        return complete;
    }
//
    public void moveWrist(boolean up,boolean down) {
        if (up || down) {
            if (up) {
                tempServoState = wristServoState.INPUT;
                Wrist.setPosition(0.7);
            }
        if (down) {
                tempServoState = wristServoState.OUTPUT;
                Wrist.setPosition(0.3);
            }
        }else {
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
        currentExtensionPower =0;
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
        telemetry.addData("CURRENT ACTION STATE",specimenHookState);
        //SPECIMEN HOOK
        if (specimenHookState == HookState.IDLE) {
            specimenHookState = HookState.POSITION_ROBOT; // Start first step

        }
        tmpExtensionPositionHolder = extensionArmMotor.getCurrentPosition();
        tmpArmPositionHolder = armMotor.getCurrentPosition();

        // Execute multi-step process based on current state
        switch (specimenHookState) {
            case POSITION_ROBOT:
                if (driveToPosition(HOOK_POS_X,HOOK_POS_Y,HOOK_DEGREES) && moveArmEncoder(tmpArmPositionHolder, DRIVE_ARM_POSITION)) {
                    resetDrivePosition();
                    specimenHookState = HookState.PLACE_ARM; // Transition to next step
                }
                break;
            case PLACE_ARM:
                if (moveArmEncoder(tmpArmPositionHolder,HOOK_ARM_HEIGHT) && moveExtensionEncoder(tmpExtensionPositionHolder,HOOK_EXTENSION_POSITION)) {
                    specimenHookState = HookState.DROP_ARM; // Transition to next step
                }
                break;
            case DROP_ARM:
                if (moveArmEncoder(tmpArmPositionHolder,HOOK_ARM_HEIGHT_2)) {
                    specimenHookState = HookState.MOVE_OUT; // Transition to next step
                    tmpActionStartTime = System.currentTimeMillis();
                    tmpExtensionPositionHolder = extensionArmMotor.getCurrentPosition();
                }
                break;
            case MOVE_OUT:
                long elapsedTime = System.currentTimeMillis() - tmpActionStartTime;
                moveIntake(false, true);
                if (elapsedTime > 300) {
                    specimenHookState = HookState.HOOK_ARM; // Transition to next step
                }
                break;
            case HOOK_ARM:
                moveIntake(false,false);
                if (moveExtensionEncoder(tmpExtensionPositionHolder,0)) {
                    specimenHookState = HookState.COMPLETE; // Transition to next step
                }
                break;
            case COMPLETE:
                setDefaultPower();
                break;
        }

    }

    public void sampleRelease() {
        telemetry.addData("CURRENT ACTION STATE",sampleReleaseState);
        //SAMPLE RELEASE TO FIRST BUCKET
        if (sampleReleaseState == releaseSampleFirstBucketState.IDLE) {
            sampleReleaseState = releaseSampleFirstBucketState.POSITION_ROBOT; // Start first step

        }
        tmpExtensionPositionHolder = extensionArmMotor.getCurrentPosition();
        tmpArmPositionHolder = armMotor.getCurrentPosition();

        // Execute multi-step process based on current state
        switch (sampleReleaseState) {
            case POSITION_ROBOT:
                if (driveToPosition(RELEASE_SAMPLE_POS_X,RELEASE_SAMPLE_POS_Y,RELEASE_SAMPLE_DEGREES) && moveArmEncoder(tmpArmPositionHolder, DRIVE_ARM_POSITION)) {
                    resetDrivePosition();
                    sampleReleaseState = releaseSampleFirstBucketState.MOVE_ARM; // Transition to next step
                }
                break;
            case MOVE_ARM:
                if (moveArmEncoder(tmpExtensionPositionHolder,RELEASE_SAMPLE_ARM_HEIGHT) && moveExtensionEncoder(tmpArmPositionHolder,RELEASE_SAMPLE_EXTENSION_POSITION)) {
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
        telemetry.addData("CURRENT ACTION STATE",samplePickupState);
        //SAMPLE PICKUP FROM GROUND
        if (samplePickupState == pickupSampleGroundState.IDLE.IDLE) {
            samplePickupState = pickupSampleGroundState.POSITION_ROBOT; // Start first step

        }
        tmpExtensionPositionHolder = extensionArmMotor.getCurrentPosition();
        tmpArmPositionHolder = armMotor.getCurrentPosition();

        // Execute multi-step process based on current state
        switch (samplePickupState) {
            case POSITION_ROBOT:
                if (driveToPosition(PICKUP_SAMPLE_POS_X, PICKUP_SAMPLE_POS_Y, PICKUP_SAMPLE_DEGREES) && moveArmEncoder(tmpArmPositionHolder, DRIVE_ARM_POSITION)) {
                    resetDrivePosition();
                    samplePickupState = pickupSampleGroundState.MOVE_ARM; // Transition to next step
                }
                break;
            case MOVE_ARM:
                if (moveArmEncoder(tmpArmPositionHolder, PICKUP_SAMPLE_ARM_HEIGHT) && moveExtensionEncoder(tmpExtensionPositionHolder, PICKUP_SAMPLE_EXTENSION_POSITION)) { // && moveWrist) {
                    tmpActionStartTime = System.currentTimeMillis();
                    samplePickupState = pickupSampleGroundState.INTAKE; // Transition to next step
                }
                break;
            case INTAKE:
                long intakeTime = System.currentTimeMillis() - tmpActionStartTime;
                boolean atPos = false;
                telemetry.addData("INTAKE", "Elapsed Time: " + intakeTime + " ms");
                // Move the intake motor
                moveIntake(true, false);

                if ((intakeTime > 5000 || touchsensor.isPressed()) && driveToPosition(PICKUP_SAMPLE_POS_INTAKE_X,PICKUP_SAMPLE_POS_Y,PICKUP_SAMPLE_DEGREES)) {
                    resetDrivePosition();
                    moveIntake(false, false);
                    if (touchsensor.isPressed()) {
                        if (moveArmEncoder(tmpArmPositionHolder, DRIVE_ARM_POSITION) && moveExtensionEncoder(tmpExtensionPositionHolder, EXTENSION_MIN_POSITION)) {
                            samplePickupState = pickupSampleGroundState.COMPLETE; // Transition to complete step
                        }
                    } else {
                        samplePickupState = pickupSampleGroundState.NOPICKUP; // Transition to the nopickup state
                    }
                }
                break;
            case NOPICKUP:
                if (driveToPosition(PICKUP_SAMPLE_POS_NOPICKUP_X, PICKUP_SAMPLE_POS_Y, PICKUP_SAMPLE_DEGREES)) {
                    resetDrivePosition();
                    long noPickupTime = System.currentTimeMillis() - tmpActionStartTime;
                    if (noPickupTime > 3000) {
                        samplePickupState = pickupSampleGroundState.POSITION_ROBOT; // Transition to next step
                    }
                }
                break;
            case COMPLETE:
                setDefaultPower();
                break;
        }
    }

    public boolean isActionRunning() {
        boolean returnVal = false;
        if (samplePickupState != pickupSampleGroundState.IDLE ||
            sampleReleaseState != releaseSampleFirstBucketState.IDLE ||
            specimenHookState != HookState.IDLE) {
            returnVal = true;
        }
        else {
            tmpDriveState = driveToPositionState.IDLE;
        }
        return returnVal;
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

    public enum releaseSampleFirstBucketState {
        IDLE,          // Waiting for button press
        POSITION_ROBOT,
        MOVE_ARM,
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
        ADJUST,
        COMPLETE
    }

}

