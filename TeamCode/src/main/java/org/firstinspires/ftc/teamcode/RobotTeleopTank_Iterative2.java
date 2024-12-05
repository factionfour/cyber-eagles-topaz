
package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

enum HookState {
    IDLE,          // Waiting for button press
    RETRACT_EXTENSION,         // First movement
    MOVE_ARM,         // Second movement
    EXTEND_EXTENSION,         // Third movement
    COMPLETE       // Process complete
}

enum HookReleaseState {
    IDLE,          // Waiting for button press
    RETRACT_EXTENSION,         // First movement
    COMPLETE       // Process complete
}

@TeleOp(name="Topaz Teleop V2 (Encoders & Improved Driving)", group="Robot")

public class RobotTeleopTank_Iterative2 extends OpMode {
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

    // Arm motor limits and power
    int ARM_MIN_POSITION =100;    // Minimum encoder position (fully retracted)
    int ARM_MAX_POSITION = 440; // Maximum encoder position (fully extended)
    double ARM_BASE_POWER = 0.3;
    double ARM_EXTRA_FORCE = 0.6;
    double ARM_MAX_SPEED = 0.8;
    double ARM_MIN_SPEED = 0.2;
    double ARM_RAMP_TICKS = 50;
    double ARM_MANUAL_MAX_SPEED = 0;

    // Extension limits and power
    int EXTENSION_MIN_POSITION = 0;    // Minimum height (fully lowered)
    int EXTENSION_MAX_POSITION = 2200; // Maximum height (fully raised)
    double EXTENSION_BASE_POWER = 0.6;

    double EXTENSION_MIN_SPEED = 0.3; // How far to move per iteration
    double EXTENSION_MAX_SPEED = 0.7; // How far to move per iteration
    double EXTENSION_RAMP_TICKS = 50;

    int MOTOR_TOLERANCE = 10; // Acceptable error in encoder ticks

    //servo limits
    double SERVO_STOPPED = 0.5;
    double SERVO_FORWARD = 1;
    double SERVO_BACKWARD = 0;

    //pre-defined positions
    int HOOK_EXTENSION_POSITION = 1800;
    int HOOK_ARM_HEIGHT = 420;
    int HOOK_RELEASE_EXTENSION_POSITION = 1400;
    int HOOK_RELEASE_ARM_HEIGHT = 300;


    int dynamicArmMinPosition = 0;
    double currentExtensionPower = 0;
    double currentArmPower = 0;
    int extensionTargetPosition = 0;
    int armTargetPosition = 0;

    HookState hookState = HookState.IDLE;
    HookReleaseState releaseState = HookReleaseState.IDLE;
    int hookArmPositionHolder = 0;
    int hookExtensionPositionHolder = 0;
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

        // Initialize arm motor
        armMotor = hardwareMap.get(DcMotor.class, "arm_1");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //armMotor.setTargetPosition(0);
        //armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //armMotor.setPower(0);

        armTargetPosition = ARM_MIN_POSITION;

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

        telemetry.addData(">", "Charlie 2 is READY.  Press START.");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        long timeHeld;

        double forward = -gamepad1.left_stick_y; // Forward/backward
        double turn = gamepad1.right_stick_x;   // Turn
        double strafe = gamepad1.left_stick_x;  // Strafe

        // Apply dead zone and smoothing
        forward = Math.abs(forward) > 0.05 ? Math.pow(forward, 3) : 0.0;
        turn = Math.abs(turn) > 0.05 ? Math.pow(turn, 3) : 0.0;
        strafe = Math.abs(strafe) > 0.05 ? Math.pow(strafe, 3) : 0.0;

        // Combine inputs for omnidirectional control
        double frontLeftPower = -forward + -turn + -strafe;
        double frontRightPower = -forward - -turn - -strafe;
        double backLeftPower = forward + -turn - -strafe;
        double backRightPower = forward - -turn + -strafe;

        // Normalize power values to avoid exceeding 1.0
        double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));

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

        // --- ARM ROTATE MOTOR CONTROL ---
        if (gamepad2.left_stick_y != 0) {  // Check if the left trigger is pulled
            telemetry.addData("ACTION", "gamepad2.left_stick_y");
            telemetry.addData("GamepadY",gamepad2.left_stick_y);
            // Get the value of the left stick Y axis (range from -1.0 to 1.0)
            float leftStickY = gamepad2.left_stick_y;

            // Invert the stick input for natural control (up = positive value, down = negative value)
            leftStickY = -leftStickY;  // If you want the arm to move up when the stick is pulled up

            // Adjust the target position based on the stick value (scaled by max speed)
            //armTargetPosition += (leftStickY * ARM_MAX_SPEED);  // Increment or decrement target position
            armTargetPosition += (leftStickY * ARM_MANUAL_MAX_SPEED);
           // Calculate dynamicArmMinPosition based on extensionPosition (to ensure the arm does not move too low)
            int extensionPosition = extensionArmMotor.getCurrentPosition();
            if (extensionPosition == EXTENSION_MAX_POSITION) {
                dynamicArmMinPosition = 200;
            } else if (extensionPosition == 900) {
                dynamicArmMinPosition = 100;
            } else if (extensionPosition == EXTENSION_MIN_POSITION) {
                dynamicArmMinPosition = 100;
            } else if (extensionPosition > 900) {
                // Linearly interpolate between 900 and 2200 for the range [100, 200]
                dynamicArmMinPosition = (int) (100 + (extensionPosition - 900) * (200 - 100) / (EXTENSION_MAX_POSITION - 900));
            } else {
                dynamicArmMinPosition = 100; // No interpolation needed as both bounds are 100
            }

            // Clamp the target position to ensure the arm doesn't exceed the boundaries
            //armTargetPosition = Math.max(armTargetPosition, ARM_MAX_POSITION);  // Clamp to max position
            //armTargetPosition = Math.min(armTargetPosition, dynamicArmMinPosition);  // Clamp to min position
            armTargetPosition = Range.clip(armTargetPosition, dynamicArmMinPosition, ARM_MAX_POSITION);
            // Set the arm motor's target position

            armMotor.setTargetPosition(armTargetPosition);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            currentArmPower = calcArmPower();
            // Set motor power based on your control system logic
            armMotor.setPower(currentArmPower);

        }

        // --- END ARM ROTATE MOTOR CONTROL ---

        // --- EXTENSION ARM MOTOR CONTROL ---

        if (gamepad2.right_stick_x != 0) {  // Check if the left trigger is pulled
            telemetry.addData("ACTION", "gamepad2.right_stick_x");
            float rightStickX = gamepad2.right_stick_x;
            extensionTargetPosition += (rightStickX * ARM_MAX_SPEED);  // Increment or decrement target position

            // Clamp the target position to ensure the arm doesn't exceed the boundaries
            extensionTargetPosition = Math.min(extensionTargetPosition, EXTENSION_MAX_POSITION);  // Clamp to max position
            extensionTargetPosition = Math.max(armTargetPosition, EXTENSION_MIN_POSITION);  // Clamp to min position

            // Set the arm motor's target position
            extensionArmMotor.setTargetPosition(extensionTargetPosition);
            extensionArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            currentExtensionPower = EXTENSION_BASE_POWER;
            extensionArmMotor.setPower(currentExtensionPower);
        }
        // --- END EXTENSION ARM MOTOR CONTROL ---

        // --- START WHEEL SERVO CONTROL ---
        if (gamepad2.dpad_left) {
            leftWheelServo.setPosition(SERVO_BACKWARD);  // Spin inward
            rightWheelServo.setPosition(SERVO_FORWARD); // Spin inward
        } else if (gamepad2.dpad_right) {
            leftWheelServo.setPosition(SERVO_FORWARD);  // Spin outward
            rightWheelServo.setPosition(SERVO_BACKWARD); // Spin outward
        } else {
            leftWheelServo.setPosition(SERVO_STOPPED);  // Neutral
            rightWheelServo.setPosition(SERVO_STOPPED); // Neutral
        }
        // --- END WHEEL SERVO CONTROL ---


        // --- AUTOMATED MOVEMENT BUTTONS

        if (gamepad2.y) {
            // If button is pressed and robot is idle, start the process
            if (hookState == HookState.IDLE) {
                hookState = HookState.RETRACT_EXTENSION; // Start first step
                hookExtensionPositionHolder = extensionArmMotor.getCurrentPosition();
            }

            // Execute multi-step process based on current state
            switch (hookState) {
                case RETRACT_EXTENSION:
                    if (moveExtensionEncoder(hookExtensionPositionHolder,0)) {
                        hookState = HookState.MOVE_ARM; // Transition to next step
                        hookArmPositionHolder = armMotor.getCurrentPosition();
                    }
                    break;
                case MOVE_ARM:
                    if (moveArmEncoder(hookArmPositionHolder,HOOK_ARM_HEIGHT)) {
                        hookState = HookState.EXTEND_EXTENSION; // Transition to next step
                        hookExtensionPositionHolder = extensionArmMotor.getCurrentPosition();
                    }
                    break;
                case EXTEND_EXTENSION:
                    if (moveExtensionEncoder(hookExtensionPositionHolder,HOOK_EXTENSION_POSITION)) {
                        hookState = HookState.COMPLETE; // Transition to next step
                    }
                    break;

                case COMPLETE:
                    armMotor.setPower(0);
                    extensionArmMotor.setPower(0);
                    break;
            }
        }
        else {
            hookState = HookState.IDLE; // Transition to next step
        }

        if (gamepad2.a) {
            //HOOK RELEASE BUTTON -- ASSUMES CURRENTLY IN HOOK POSITION
            // If button is pressed and robot is idle, start the process
            if (releaseState == HookReleaseState.IDLE) {
                releaseState = HookReleaseState.RETRACT_EXTENSION; // Start first step
                hookExtensionPositionHolder = extensionArmMotor.getCurrentPosition();
                hookArmPositionHolder = armMotor.getCurrentPosition();
            }

            // Execute multi-step process based on current state
            switch (hookState) {
                case RETRACT_EXTENSION:
                    if (moveExtensionEncoder(hookExtensionPositionHolder,HOOK_RELEASE_EXTENSION_POSITION) &&
                        moveArmEncoder(hookArmPositionHolder,HOOK_RELEASE_ARM_HEIGHT)) {
                        releaseState = HookReleaseState.COMPLETE; // Transition to next step
                    }
                    break;

                case COMPLETE:
                    armMotor.setPower(0);
                    extensionArmMotor.setPower(0);
                    break;
            }

        }
        else {
            releaseState = HookReleaseState.IDLE; // Transition to next step
        }

        // --- END AUTOMATED MOVEMENT BUTTONS

        // --- STOP & EMERGENCY ACTIONS

        //EMERGENCY RESET BUTTON (BOTTOM)
        if (gamepad2.back) {// || (extensionTargetPosition <= 0 && extensionArmMotor.getCurrentPosition() > 0 && (System.currentTimeMillis() - lastExtensionMovementTime) > 500)) {
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

        // --- END EMERGENCY ACTIONS

        // Send telemetry message to signify robot running;
        telemetry.addData("front",  "%.2f", forward);
        telemetry.addData("turn", "%.2f", turn);
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

    private Boolean moveExtensionEncoder(int startPosition, int targetPosition) {
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

    public double calcArmPower() {
        // Adjust arm motor power based on vertical arm position
        double horizontalArmPower = ARM_BASE_POWER;
        double verticalFactor = (double) extensionTargetPosition / EXTENSION_MAX_POSITION;
        horizontalArmPower += verticalFactor * ARM_EXTRA_FORCE; // Add extra power when fully raised
        horizontalArmPower = Range.clip(horizontalArmPower, 0.2, 1.0);
        return horizontalArmPower;
    }

    public double getAngle() {
        double angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        return angle;
    }
}
