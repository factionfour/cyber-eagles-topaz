
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

@TeleOp(name="Topaz Teleop Test V2 (Encoders & Improved Driving)", group="Robot")

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
    double ARM_PRECISION_SPEED = .1;
    double ARM_BASE_SPEED = .3;
    double ARM_MAX_SPEED = 0.7;
    int ARM_RAMP_TIME = 200;

    // Vertical extension limits and base power

    int EXTENSION_MIN_POSITION = 0;    // Minimum height (fully lowered)
    int EXTENSION_MAX_POSITION = 2200; // Maximum height (fully raised)
    double EXTENSION_PRESCION_SPEED = .1;
    double EXTENSION_BASE_SPEED = .3; // How far to move per iteration
    double EXTENSION_MAX_SPEED = .9; // How far to move per iteration
    int EXTENSION_RAMP_TIME = 200;


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
    long lastXPressTime = 0;
    long lastBPressTime = 0;
    long lastDPadUpPressTime = 0;
    long lastDPadDownPressTime = 0;

    double currentExtensionSpeed = 0;
    double currentArmSpeed = 0;
    int extensionTargetPosition = 0;
    int armTargetPosition = 0;

    HookState hookState = HookState.IDLE;
    HookReleaseState releaseState = HookReleaseState.IDLE;

    private ElapsedTime stateTimer = new ElapsedTime();
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
        //extensionArmMotor.setTargetPosition(0);
        //extensionArmMotor.setPower(ARM_BASE_SPEED);
        //extensionArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        double frontLeftPower = forward + turn + strafe;
        double frontRightPower = forward - turn - strafe;
        double backLeftPower = forward + turn - strafe;
        double backRightPower = forward - turn + strafe;

        // Normalize power values to avoid exceeding 1.0
        double maxPower = Math.max(1.0, Math.abs(frontLeftPower));
        frontLeftPower /= maxPower;
        frontRightPower /= maxPower;
        backLeftPower /= maxPower;
        backRightPower /= maxPower;

        // Set motor powers
        frontleftDrive.setPower(frontLeftPower);
        frontrightDrive.setPower(frontRightPower);
        backleftDrive.setPower(backLeftPower);
        backrightDrive.setPower(backRightPower);

//        frontleftDrive.setPower(front);
//        frontrightDrive.setPower(front);
//        backleftDrive.setPower(-front);
//        backrightDrive.setPower(-front);
//
//        frontleftDrive.setPower(-turn);
//        frontrightDrive.setPower(turn);
//        backleftDrive.setPower(-turn);
//        backrightDrive.setPower(turn);
//
//        frontleftDrive.setPower(-strafe);
//        frontrightDrive.setPower(strafe);
//        backleftDrive.setPower(strafe);
//        backrightDrive.setPower(-strafe);

        long currentTime = System.currentTimeMillis();

        // --- ARM ROTATE MOTOR CONTROL ---
        if (gamepad2.dpad_up) {
            telemetry.addData("ACTION", "gamepad2.dpad_up");

            timeHeld = currentTime - lastDPadUpPressTime;

            // Ramp power over time if the button is held
            if (timeHeld < ARM_RAMP_TIME) {
                currentArmSpeed = ARM_MAX_SPEED * (timeHeld / (float) ARM_RAMP_TIME); // Gradual ramp up
            } else {
                currentArmSpeed = ARM_MAX_SPEED; // Max power after ramp time
            }

            // Clamp power to maximum
            currentArmSpeed = Math.min(currentArmSpeed, ARM_MAX_SPEED);

            // Get current position
            int currentPosition = armMotor.getCurrentPosition();

            // Ensure the motor doesn't move beyond max or min positions
            if (currentPosition < ARM_MAX_POSITION) {
                // Allow movement upwards
                armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                armMotor.setPower(currentArmSpeed);
            } else {
                // Stop motor if at max position
                currentArmSpeed = 0;
                armMotor.setPower(currentArmSpeed);
            }

            lastDPadUpPressTime = currentTime;
        } else {
            lastDPadUpPressTime = 0;
            currentArmSpeed = 0;
            armMotor.setPower(currentArmSpeed);
        }



        if (gamepad2.dpad_down) {
            telemetry.addData("ACTION", "gamepad2.dpad_down");
            int extensionPosition = extensionArmMotor.getCurrentPosition();

            // Calculate dynamicArmMinPosition based on extensionPosition
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

            // Determine how long the button has been held
            timeHeld = currentTime - lastDPadDownPressTime;

            // Calculate ramped power
            if (timeHeld < ARM_RAMP_TIME) {
                currentArmSpeed = -(ARM_MAX_SPEED * (timeHeld / (float) ARM_RAMP_TIME)); // Ramp power gradually
            } else {
                currentArmSpeed = -ARM_MAX_SPEED; // Max power after ramp time
            }

            // Clamp power to maximum
            currentArmSpeed = Math.max(currentArmSpeed, -ARM_MAX_SPEED);

            // Get current position and calculate new position bounds
            int currentPosition = armMotor.getCurrentPosition();
            if (currentPosition > dynamicArmMinPosition) {
                // Allow movement downwards
                armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                armMotor.setPower(currentArmSpeed); // Negative power to move downward
            } else {
                currentArmSpeed = 0;
                armMotor.setPower(currentArmSpeed);
            }
            lastDPadDownPressTime = currentTime;
        } else {
            lastDPadDownPressTime = 0;
            currentArmSpeed = 0;
            armMotor.setPower(currentArmSpeed);
        }


        // --- END ARM ROTATE MOTOR CONTROL ---

        // --- EXTENSION ARM MOTOR CONTROL ---

        if (gamepad2.x) {
            telemetry.addData("ACTION", "gamepad2.x");

            timeHeld = currentTime - lastXPressTime; // Time the X button has been held

            // Ramp speed over time if the button is held
            if (timeHeld < EXTENSION_RAMP_TIME) {
                // Gradually increase the power based on how long the button is held
                currentExtensionSpeed = EXTENSION_MAX_SPEED * (timeHeld / (double) EXTENSION_RAMP_TIME);
            } else {
                // Once ramp time has passed, use max power
                currentExtensionSpeed = EXTENSION_MAX_SPEED;
            }

            currentExtensionSpeed = Math.min(currentExtensionSpeed, EXTENSION_MAX_SPEED);

            int currentPosition = extensionArmMotor.getCurrentPosition();

            // Only allow movement if within limits
            if (currentPosition < EXTENSION_MAX_POSITION) {
                extensionArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                extensionArmMotor.setPower(currentExtensionSpeed);
            } else {
                currentExtensionSpeed = 0;
                extensionArmMotor.setPower(currentExtensionSpeed); // Stop motor if at max
            }

            // Update the last press time
            lastXPressTime = currentTime;
        } else {
            currentExtensionSpeed = 0;
            extensionArmMotor.setPower(currentExtensionSpeed); // Stop motor if at max
            lastXPressTime = 0;
        }

        if (gamepad2.b) {
            telemetry.addData("ACTION", "gamepad2.b");

            timeHeld = currentTime - lastXPressTime; // Time the X button has been held

            // Ramp speed over time if the button is held
            if (timeHeld < EXTENSION_RAMP_TIME) {
                // Gradually increase the power based on how long the button is held
                currentExtensionSpeed = -(EXTENSION_MAX_SPEED * (timeHeld / (double) EXTENSION_RAMP_TIME));
            } else {
                // Once ramp time has passed, use max power
                currentExtensionSpeed = -EXTENSION_MAX_SPEED;
            }

            currentExtensionSpeed = Math.min(currentExtensionSpeed, EXTENSION_MAX_SPEED);
            int currentPosition = extensionArmMotor.getCurrentPosition();

            // Only allow movement if within limits
            if (currentPosition > EXTENSION_MIN_POSITION) {
                extensionArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                extensionArmMotor.setPower(currentExtensionSpeed);
            } else {
                currentExtensionSpeed = 0;
                extensionArmMotor.setPower(currentExtensionSpeed); // Stop motor if at max
            }
            // Update the last press time
            lastXPressTime = currentTime;
        } else {
            currentExtensionSpeed = 0;
            extensionArmMotor.setPower(currentExtensionSpeed); // Stop motor if at max
            lastXPressTime = 0;
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
            if (hookState == HookState.IDLE || hookState == HookState.COMPLETE) {
                hookState = HookState.RETRACT_EXTENSION; // Start first step
                stateTimer.reset();
            }

            // Execute multi-step process based on current state
            switch (hookState) {
                case RETRACT_EXTENSION:
                    moveExtension(0);
                    if (stateTimer.seconds() >= 1.5) {
                        hookState = HookState.MOVE_ARM; // Transition to next step
                    }
                    break;
                case MOVE_ARM:
                    moveArmEncoder(HOOK_ARM_HEIGHT);
                    if (stateTimer.seconds() >= 3.5) {
                        hookState = HookState.EXTEND_EXTENSION; // Transition to next step
                    }
                    break;
                case EXTEND_EXTENSION:
                    moveExtension(HOOK_EXTENSION_POSITION);
                    if (stateTimer.seconds() >= 5.5) {
                        hookState = HookState.COMPLETE; // Transition to next step
                    }
                    break;

                case COMPLETE:
                    armMotor.setPower(0);
                    extensionArmMotor.setPower(0);
                    //hookState = HookState.IDLE;
                    break;
            }
        }

        if (gamepad2.a) {
            //HOOK RELEASE BUTTON -- ASSUMES CURRENTLY IN HOOK POSITION
            // If button is pressed and robot is idle, start the process
            if (releaseState == HookReleaseState.IDLE || releaseState == HookReleaseState.COMPLETE) {
                releaseState = HookReleaseState.RETRACT_EXTENSION; // Start first step
                stateTimer.reset();
            }

            // Execute multi-step process based on current state
            switch (hookState) {
                case RETRACT_EXTENSION:
                    moveExtension(HOOK_RELEASE_EXTENSION_POSITION);
                    moveArmEncoder(HOOK_ARM_HEIGHT);
                    if (stateTimer.seconds() >= 1) {
                        releaseState = HookReleaseState.COMPLETE; // Transition to next step
                    }
                    break;

                case COMPLETE:
                    armMotor.setPower(0);
                    extensionArmMotor.setPower(0);
                    //releaseState = HookReleaseState.IDLE;
                    break;
            }
        }

        // --- END AUTOMATED MOVEMENT BUTTONS

        // --- STOP & EMERGENCY ACTIONS

        // Reset the arm position scale down the motor if it hasn't moved in 1 second (and it should be)


        //EMERGENCY STOP BUTTON (BOTTOM)
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
        telemetry.addData("Arm Calculated Speed", currentArmSpeed);
        telemetry.addData("Arm Motor Busy", armMotor.isBusy());
        telemetry.addData("Extension Current Position", extensionArmMotor.getCurrentPosition());
        telemetry.addData("Extension Target Position", extensionTargetPosition);
        telemetry.addData("Extension Calculated Speed", currentExtensionSpeed);
        telemetry.addData("Extension Motor Busy", extensionArmMotor.isBusy());
        telemetry.addData("Left Servo Position", leftWheelServo.getPosition());
        telemetry.addData("Right Servo Position", rightWheelServo.getPosition());


    }

    private void moveExtension(int targetPosition) {
        extensionTargetPosition = targetPosition;
        extensionArmMotor.setTargetPosition(extensionTargetPosition);
        extensionArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        currentExtensionSpeed = EXTENSION_MAX_SPEED;
        extensionArmMotor.setPower(currentExtensionSpeed);
    }

    //raise or lower the robot's arm to a specific height
    public void moveArmEncoder(int targetPosition) {

        armTargetPosition = targetPosition;
        armMotor.setTargetPosition(armTargetPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        currentArmSpeed = ARM_MAX_SPEED;
        armMotor.setPower(currentArmSpeed);

    }

    public double getAngle() {
        double angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        return angle;
    }
}
