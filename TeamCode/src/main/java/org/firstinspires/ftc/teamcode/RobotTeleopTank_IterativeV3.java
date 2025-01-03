
package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Topaz Teleop V3", group="Robot")
public class RobotTeleopTank_IterativeV3 extends OpMode {
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
    public Servo Wrist;

    double DRIVING_SLOW =0.5;

    // Arm motor limits and power
    int ARM_MIN_POSITION =100;    // Minimum encoder position (fully retracted)
    int ARM_MAX_POSITION = 850; // Maximum encoder position (fully extended)
    double ARM_BASE_POWER = 0.2;
    double ARM_EXTRA_FORCE = 0.01;
    double ARM_MAX_SPEED = 0.8;
    double ARM_MIN_SPEED = 0.2;
    double ARM_RAMP_TICKS = 50;
    double ARM_MANUAL_MAX_SPEED = 40;

    // Extension limits and power
    int EXTENSION_MIN_POSITION = 0;    // Minimum height (fully lowered)
    int EXTENSION_MAX_POSITION = 2200; // Maximum height (fully raised)
    double EXTENSION_BASE_POWER = 0.3;
    double EXTENSION_EXTRA_FORCE = 0.6;

    double EXTENSION_MIN_SPEED = 0.3; // How far to move per iteration
    double EXTENSION_MAX_SPEED = 0.7; // How far to move per iteration
    double EXTENSION_RAMP_TICKS = 50;
    double EXTENSION_MANUAL_MAX_SPEED = 100;

    //tolernances
    int MOTOR_TOLERANCE = 10; // Acceptable error in encoder ticks
    double POSITION_TOLERANCE_CM = 2.0;
    double HEADING_TOLERANCE_RADIANS = 0.05; // Approximately 2.9 degrees

    //servo limits
    double SERVO_STOPPED = 0.5;
    double SERVO_FORWARD = 1;
    double SERVO_BACKWARD = 0;

    //pre-defined positions
    int HOOK_EXTENSION_POSITION = 1800;
    int HOOK_ARM_HEIGHT = 750;
    int HOOK_RADIANS = 0;
    int HOOK_POS_X = 550;
    int HOOK_POS_Y = 200;

    int HOOK_RELEASE_EXTENSION_POSITION = 1400;
    int HOOK_RELEASE_ARM_HEIGHT = 670;
    int HOOK_RELEASE_RADIANS = 0;
    int HOOK_RELEASE_POS_X = 550;
    int HOOK_RELEASE_POS_Y = 200;

    int PICKUP_SPECIMEN_EXTENSION_POSITION = 600;
    int PICKUP_SPECIMEN_ARM_HEIGHT = 150;
    int PICKUP_SPECIMEN_RADIANS = 0;
    int PICKUP_SPECIMEN_POS_X = 550;
    int PICKUP_SPECIMEN_POS_Y = 200;

    int PICKUP_SAMPLE_ARM_HEIGHT = 150;
    int PICKUP_SAMPLE_EXTENSION_POSITION = 800;
    int PICKUP_SAMPLE_RADIANS = 0;
    int PICKUP_SAMPLE_POS_X = 550;
    int PICKUP_SAMPLE_POS_Y = 200;

    int RELEASE_SAMPLE_ARM_HEIGHT = 400;
    int RELEASE_SAMPLE_EXTENSION_POSITION = 2000;
    int RELEASE_SAMPLE_RADIANS = 0;
    int RELEASE_SAMPLE_POS_X = 550;
    int RELEASE_SAMPLE_POS_Y = 200;

    int dynamicArmMinPosition = 0;
    double currentExtensionPower = 0;
    double currentArmPower = 0;
    int extensionTargetPosition = 0;
    int armTargetPosition = 0;

    double currentForward;
    double currentTurn;
    double currentStrafe;

    manualArmState tmpArmState = manualArmState.IDLE;
    manualExtensionState tmpExtensionState = manualExtensionState.IDLE;
    HookState specimenHookState = HookState.IDLE;
    HookReleaseState specimenReleaseState = HookReleaseState.IDLE;
    manualServoState tmpServoState = manualServoState.IDLE;

    pickupSampleGroundState samplePickupState = pickupSampleGroundState.IDLE;
    releaseSampleFirstBucketState sampleReleaseState = releaseSampleFirstBucketState.IDLE;

    int tmpArmPositionHolder = 0;
    int tmpExtensionPositionHolder = 0;
    long tmpActionStartTime = 0;
    RobotPositionTracker positionTracker;

    @Override
    public void init() {
        // Define and Initialize wheel Motors
        frontleftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontrightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backrightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        backleftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");

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
        leftWheelServo.setPosition(0.5); // Neutral position
        rightWheelServo.setPosition(0.5); // Neutral position

        imu = hardwareMap.get(IMU.class,"imu");
        imu.initialize((new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.LEFT))));

        positionTracker = new RobotPositionTracker(hardwareMap.get(GoBildaPinpointDriver.class,"odo"),hardwareMap.get(IMU.class, "imu"));
        telemetry.addData(">", "Charlie 3 is READY.  Press START.");    //
    }
    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        long currentTime = System.currentTimeMillis();

        driveWheels(gamepad1.left_stick_y,gamepad1.right_stick_x,gamepad1.left_stick_x);
//        moveIntake(gamepad2.left_bumper,gamepad2.right_bumper);
        moveArm(gamepad2.left_stick_y);
        moveExtension(gamepad2.right_stick_y);
        moveWrist(gamepad2.left_bumper,gamepad2.right_bumper);

        // --- STOP & EMERGENCY ACTIONS
        if (gamepad2.back) {
            emergencyReset();
        }

        if (gamepad2.y) {
            specimenHook();
        }
        else {
            specimenHookState = HookState.IDLE; // Transition to next step
        }

        if (gamepad2.y) {
            specimenHook();
        }
        else {
            specimenHookState = HookState.IDLE; // Transition to next step
        }

        if (gamepad2.b) {
            sampleRelease();
        }
        else {
            sampleReleaseState = releaseSampleFirstBucketState.IDLE; // Transition to next step
        }

        if (gamepad2.a) {
            samplePickupGround();
        }
        else {
            if (samplePickupState != pickupSampleGroundState.IDLE) {
                moveIntake(false,false);
                samplePickupState = samplePickupState.IDLE;
            }
        }

        addTelemetry();
    }



    public void addTelemetry() {
        telemetry.addData("X",  "%.2f", positionTracker.getXPositionCM());
        telemetry.addData("Y",  "%.2f", positionTracker.getYPositionCM());
        telemetry.addData("front",  "%.2f", currentForward);
        telemetry.addData("turn", "%.2f", currentTurn);
        telemetry.addData("strafe", "%.2f", currentStrafe);
        telemetry.addData("Arm Motor Position", armMotor.getCurrentPosition());
        telemetry.addData("Arm Target Position", armTargetPosition);
        telemetry.addData("Arm Calculated Min Position", dynamicArmMinPosition);
        telemetry.addData("Arm Calculated Power", currentArmPower);
        telemetry.addData("Arm Motor Busy", armMotor.isBusy());
        telemetry.addData("Extension Current Position", extensionArmMotor.getCurrentPosition());
        telemetry.addData("Extension Target Position", extensionTargetPosition);
        telemetry.addData("Extension Calculated Power", currentExtensionPower);
        telemetry.addData("Extension Motor Busy", extensionArmMotor.isBusy());
        telemetry.addData("Left Servo Position", leftWheelServo.getPosition());
        telemetry.addData("Right Servo Position", rightWheelServo.getPosition());
        telemetry.update();
    }

    public void driveWheels(double tmpForward, double tmpTurn, double tmpStrafe) {
        currentForward = tmpForward;
        currentTurn = tmpTurn;
        currentStrafe = tmpStrafe;

        // Apply dead zone and smoothing
        currentForward = Math.abs(currentForward) > 0.05 ? Math.pow(currentForward, 3) : 0.0;
        currentTurn = Math.abs(currentTurn) > 0.05 ? Math.pow(currentTurn, 3) : 0.0;
        currentStrafe = Math.abs(currentStrafe) > 0.05 ? Math.pow(currentStrafe, 3) : 0.0;

        // Combine inputs for omnidirectional control
        double frontLeftPower = -currentForward + -currentTurn + -currentStrafe;
        double frontRightPower = -currentForward - -currentTurn - -currentStrafe;
        double backLeftPower = currentForward + -currentTurn - -currentStrafe;
        double backRightPower = currentForward - -currentTurn + -currentStrafe;

        // Normalize power values to avoid exceeding 1.0
        double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));

        maxPower = maxPower * DRIVING_SLOW;
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

        long currentTime = System.currentTimeMillis();

    }

    public boolean moveToPosition(double targetXCM, double targetYCM, double targetHeadingRadians) {
        // Update the robot's current position and heading
        positionTracker.updatePosition();
        double currentX = positionTracker.getXPositionCM();
        double currentY = positionTracker.getYPositionCM();
        double currentHeading = positionTracker.getHeading();
        boolean atTargetPos = false;
        boolean atTargetRad = false;
        // Calculate differences to target
        double deltaX = targetXCM - currentX;
        double deltaY = targetYCM - currentY;
        double distanceToTarget = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        double deltaHeading = targetHeadingRadians - currentHeading;

        // Normalize deltaHeading to [-π, π]
        if (deltaHeading > Math.PI) deltaHeading -= 2 * Math.PI;
        if (deltaHeading < -Math.PI) deltaHeading += 2 * Math.PI;

        // Step 1: Move to the target position
        if (distanceToTarget > POSITION_TOLERANCE_CM) {
            // Calculate power components for forward and strafe motion
            double forwardPower = calculateDrivePower(deltaY); // Forward is Y-axis
            double strafePower = calculateDrivePower(deltaX);  // Strafe is X-axis

            // Normalize power values to ensure they don't exceed the max allowed
            double maxPower = Math.max(Math.abs(forwardPower), Math.abs(strafePower));
            if (maxPower > 1.0) {
                forwardPower /= maxPower;
                strafePower /= maxPower;
            }

            // Drive the robot with both forward and strafe power
            driveWheels(forwardPower, 0, strafePower);
            return false;
        }
        else {
            atTargetPos = true;
        }

        // Stop the robot once the position is reached
        driveWheels(0, 0, 0);

        // Step 2: Turn to the target heading
        if (Math.abs(deltaHeading) > HEADING_TOLERANCE_RADIANS) {
            // Calculate turn power (proportional control)
            double turnPower = calculateTurnPower(deltaHeading);

            // Drive the robot with turn power
            driveWheels(0, turnPower, 0);
        }
        else {
            atTargetRad = true;
        }
        if (atTargetPos && atTargetRad) {
            return true;
        }
        else {
            return false;
        }

    }


    // Helper method to calculate drive power based on distance
    private double calculateDrivePower(double distance) {
        double maxPower = 1.0; // Maximum power
        double minPower = 0.2; // Minimum power for precision
        double kDrive = 0.05;  // Proportional control factor

        // Calculate power proportional to distance
        double power = Math.max(minPower, Math.min(maxPower, Math.abs(distance) * kDrive));

        // Set the correct sign for direction
        return (distance < 0) ? -power : power;
    }

    // Helper method to calculate turn power based on heading difference
    private double calculateTurnPower(double deltaHeading) {
        double maxTurnPower = 0.8; // Maximum turning power
        double minTurnPower = 0.2; // Minimum turning power for precision
        double kTurn = 0.5;        // Proportional control factor for turning

        // Calculate turn power proportional to heading difference
        double power = Math.max(minTurnPower, Math.min(maxTurnPower, Math.abs(deltaHeading) * kTurn));

        // Set the correct sign for turn direction
        return (deltaHeading < 0) ? -power : power;
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
                dynamicArmMinPosition = 100;
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
                armMotor.setPower(calcArmPower());
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

    //raise or lower the robot's arm to a specific height
    public Boolean moveArmEncoder(int startPosition, int targetPosition) {
        Boolean complete = false;
        int currentPos = armMotor.getCurrentPosition();
        int distanceToTarget = Math.abs(targetPosition - currentPos);
        int distanceFromStart = Math.abs(currentPos - startPosition);
        double currentSpeed = 0;
        if (currentPos >= (targetPosition - MOTOR_TOLERANCE) && currentPos <= (targetPosition + MOTOR_TOLERANCE)) {
            complete = true;
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

    public void moveWrist(boolean up,boolean down) {
        if (up || down) {
            if (up) {
                Wrist.setPosition(SERVO_FORWARD);

            }
        if (down) {
                Wrist.setPosition(SERVO_BACKWARD);

            }
        }else {
        if (tmpServoState == manualServoState.INPUT || tmpServoState == manualServoState.OUTPUT) {
            tmpServoState = manualServoState.IDLE;
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
        currentArmPower = calcArmPower();
        armMotor.setPower(currentArmPower);

        extensionArmMotor.setTargetPosition(extensionTargetPosition);
        extensionArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        currentExtensionPower =0;
        extensionArmMotor.setPower(currentExtensionPower);
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

    private void specimenHook() {

        //SPECIMEN HOOK
        if (specimenHookState == HookState.IDLE) {
            specimenHookState = HookState.POSITION_ROBOT; // Start first step
            tmpExtensionPositionHolder = extensionArmMotor.getCurrentPosition();
            tmpArmPositionHolder = armMotor.getCurrentPosition();
        }

        // Execute multi-step process based on current state
        switch (specimenHookState) {
            case POSITION_ROBOT:
                if (moveToPosition(HOOK_POS_X,HOOK_POS_Y,HOOK_RADIANS)) {
                    specimenHookState = HookState.PLACE_ARM; // Transition to next step
                }
                break;
            case PLACE_ARM:
                if (moveArmEncoder(tmpArmPositionHolder,HOOK_ARM_HEIGHT) && moveExtensionEncoder(tmpExtensionPositionHolder,HOOK_EXTENSION_POSITION)) {
                    specimenHookState = HookState.MOVE_OUT; // Transition to next step
                    tmpActionStartTime = System.currentTimeMillis();
                    tmpExtensionPositionHolder = extensionArmMotor.getCurrentPosition();
                }
                break;
            case MOVE_OUT:
                long elapsedTime = System.currentTimeMillis() - tmpActionStartTime;
                if (elapsedTime <= 500) {
                    moveIntake(false, true);
                }
                else {
                    moveIntake(false,false);
                    specimenHookState = HookState.HOOK_ARM; // Transition to next step
                }
            case HOOK_ARM:
                if (moveExtensionEncoder(tmpExtensionPositionHolder,0)) {
                    specimenHookState = HookState.COMPLETE; // Transition to next step
                }
                break;
            case COMPLETE:
                setDefaultPower();
                break;
        }

    }

    private void sampleRelease() {
        //SAMPLE RELEASE TO FIRST BUCKET
        if (sampleReleaseState == releaseSampleFirstBucketState.IDLE) {
            sampleReleaseState = releaseSampleFirstBucketState.POSITION_ROBOT; // Start first step
            tmpExtensionPositionHolder = extensionArmMotor.getCurrentPosition();
            tmpArmPositionHolder = armMotor.getCurrentPosition();
        }

        // Execute multi-step process based on current state
        switch (sampleReleaseState) {
            case POSITION_ROBOT:
                if (moveToPosition(RELEASE_SAMPLE_POS_X,RELEASE_SAMPLE_POS_Y,RELEASE_SAMPLE_RADIANS)) {
                    //TODO: BUILD CODE TO MOVE ROBOT TO SPECIFIC POSITION IN MAP BASED ON TELEMETRY
                    sampleReleaseState = releaseSampleFirstBucketState.MOVE_ARM; // Transition to next step
                }
                break;
            case MOVE_ARM:
                if (moveArmEncoder(tmpExtensionPositionHolder,RELEASE_SAMPLE_ARM_HEIGHT) && moveExtensionEncoder(tmpArmPositionHolder,RELEASE_SAMPLE_EXTENSION_POSITION)) {
                    sampleReleaseState = releaseSampleFirstBucketState.COMPLETE; // Transition to next step
                    tmpExtensionPositionHolder = extensionArmMotor.getCurrentPosition();
                }
                break;
            case COMPLETE:
                setDefaultPower();
                break;
        }
    }

    private void samplePickupGround() {
        //SAMPLE PICKUP FROM GROUND
        if (samplePickupState == pickupSampleGroundState.IDLE.IDLE) {
            samplePickupState = pickupSampleGroundState.POSITION_ROBOT; // Start first step
            tmpExtensionPositionHolder = extensionArmMotor.getCurrentPosition();
            tmpArmPositionHolder = armMotor.getCurrentPosition();
        }

        // Execute multi-step process based on current state
        switch (samplePickupState) {
            case POSITION_ROBOT:
                if (moveToPosition(PICKUP_SAMPLE_POS_X,PICKUP_SAMPLE_POS_Y,PICKUP_SAMPLE_RADIANS)) {
                    samplePickupState = pickupSampleGroundState.MOVE_ARM; // Transition to next step
                }
                break;
            case MOVE_ARM:
                if (moveArmEncoder(tmpExtensionPositionHolder,PICKUP_SAMPLE_ARM_HEIGHT) && moveExtensionEncoder(tmpExtensionPositionHolder,0)) {
                    samplePickupState = pickupSampleGroundState.INTAKE; // Transition to next step
                }
                break;
            case INTAKE:
                moveIntake(false,false);
                break;
        }
    }

    public enum HookState {
        IDLE,          // Waiting for button press
        POSITION_ROBOT,
        PLACE_ARM,         // Second movement
        MOVE_OUT,
        HOOK_ARM,
        COMPLETE       // Process complete
    }

    public enum HookReleaseState {
        IDLE,          // Waiting for button press
        RETRACT_EXTENSION,         // First movement
        COMPLETE       // Process complete
    }

    public enum pickupSampleGroundState {
        IDLE,          // Waiting for button press
        POSITION_ROBOT,
        MOVE_ARM,
        INTAKE
    }

    public enum releaseSampleFirstBucketState {
        IDLE,          // Waiting for button press
        POSITION_ROBOT,
        MOVE_ARM,
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




}

