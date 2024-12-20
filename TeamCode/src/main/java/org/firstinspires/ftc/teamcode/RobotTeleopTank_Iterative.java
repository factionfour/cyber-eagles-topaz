/*

package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.text.DecimalFormat;
@Disabled
@TeleOp(name="Z: Topaz Teleop OLD", group="Robot")

public class RobotTeleopTank_Iterative extends OpMode {
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
    public DcMotor linearExtender;

    // Arm motor limits and power
    int armTargetPosition = 0;
    int ARM_MIN_POSITION =100;    // Minimum encoder position (fully retracted)
    int ARM_MAX_POSITION = 440; // Maximum encoder position (fully extended)
    double ARM_BASE_POWER = 0.3;
    double ARM_EXTRA_FORCE = 0.6;
    int ARM_MAX_SPEED = 8; // How far to move per loop iteration
    int ARM_MIN_POSITION_WHEN_EXTENSION_EXTENDED = 150;

    // Vertical extension limits and base power
    int extensionTargetPosition = 0;
    int EXTENSION_MIN_POSITION = 0;    // Minimum height (fully lowered)
    int EXTENSION_MAX_POSITION = 2200; // Maximum height (fully raised)
    int EXTENSION_MAX_SPEED = 125; // How far to move per iteration
    int EXTENSION_RAMP_TIME = 100;
    int EXTENSION_PRESCION_SPEED = 60;
    int RAMP_TIME = 200;

    double EXTENSION_BASE_POWER = 0.5; // Base power to hold position
    double LINEAR_BASE_POWER = 0.5; // Base power to hold position
    double EXTENSION_EXTRA_FORCE = 0.4; // Extra power when arm is extended
    long DEBOUNCE_DELAY = 40; // Time in milliseconds to wait before accepting another press
    int MOTOR_TOLERANCE = 10; // Acceptable error in encoder ticks

    //servo limits
    double SERVO_STOPPED = 0.5;
    double SERVO_FORWARD = 1;
    double SERVO_BACKWARD = 0;

    //pre-defined positions
    int HOOK_EXTENSION_POSITION = 1800;
    int HOOK_ARM_HEIGHT = 420;
    long HOOK_ACTION_TIMEOUT = 4000; // 5 seconds
    int HOOK_RELEASE_EXTENSION_POSITION = 1400;
    int HOOK_RELEASE_ARM_HEIGHT = 300;
    long HOOK_RELEASE_ACTION_TIMEOUT = 1500; //1.5 seconds

    long lastArmMovementTime = System.currentTimeMillis(); // Last time the arm moved
    boolean armMoveTriggered = false; // Flag to indicate if the arm moved
    int dynamicArmMinPosition = 0;
    long lastXPressTime = 0;
    long lastBPressTime = 0;

    long lastDPadUpPressTime = 0;
    long lastDPadDownPressTime = 0;

    double currentExtensionPower = 0;
    double currentArmPower = 0;
    boolean predefinedActionRunning = false;
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
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0);

        armTargetPosition = ARM_MIN_POSITION;

        // Initialize extension arm motor
        extensionArmMotor = hardwareMap.get(DcMotor.class, "arm_extendo");
        extensionArmMotor.setDirection(DcMotor.Direction.REVERSE);
        extensionArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extensionArmMotor.setTargetPosition(0);
        extensionArmMotor.setPower(EXTENSION_BASE_POWER);
        extensionArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        linearExtender = hardwareMap.get(DcMotor.class, "linear_extendo");
        linearExtender.setDirection(DcMotor.Direction.FORWARD);
        linearExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearExtender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearExtender.setTargetPosition(0);
        linearExtender.setPower(LINEAR_BASE_POWER);
        linearExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        // Initialize wheel servos
        leftWheelServo = hardwareMap.get(Servo.class, "servo_one");
        rightWheelServo = hardwareMap.get(Servo.class, "servo_two");
        leftWheelServo.setPosition(0.5); // Neutral position
        rightWheelServo.setPosition(0.5); // Neutral position

        imu = hardwareMap.get(IMU.class,"imu");
        imu.initialize((new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.LEFT))));

        telemetry.addData(">", "Charlie 2 is READY.  Press START.");    //
    }

    */
/*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     *//*

    @Override
    public void init_loop() {

    }

    */
/*
     * Code to run ONCE when the driver hits START
     *//*

    @Override
    public void start() {

    }

    */
/*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     *//*

    @Override
    public void loop() {
        double front;
        double turn;
        double strafe;

        // Run wheels in tank mode (note: The joystick goes negative when pushed forward, so negate it)
        front = gamepad1.left_stick_y;
        turn = gamepad1.right_stick_x;
        strafe = gamepad1.left_stick_x;

        frontleftDrive.setPower(front);
        frontrightDrive.setPower(front);
        backleftDrive.setPower(-front);
        backrightDrive.setPower(-front);

        frontleftDrive.setPower(-turn);
        frontrightDrive.setPower(turn);
        backleftDrive.setPower(-turn);
        backrightDrive.setPower(turn);

        frontleftDrive.setPower(-strafe);
        frontrightDrive.setPower(strafe);
        backleftDrive.setPower(strafe);
        backrightDrive.setPower(-strafe);

        long currentTime = System.currentTimeMillis();

        // --- ARM ROTATE MOTOR CONTROL ---
        if (gamepad2.dpad_up && !predefinedActionRunning) {
            telemetry.addData("ACTION", "gamepad2.dpad_up");
            //if (currentTime - lastDPadUpPressTime > DEBOUNCE_DELAY) {
                //long timeHeld = currentTime - lastDPadUpPressTime;

                //if (timeHeld < RAMP_TIME) {
                //    armTargetPosition += Math.min((int) (ARM_MAX_SPEED * (timeHeld / (float) RAMP_TIME)), ARM_MAX_SPEED);
                //} else {
                    armTargetPosition += ARM_MAX_SPEED;  // Max speed after ramp time
                //}
                armTargetPosition = Math.min(armTargetPosition, ARM_MAX_POSITION);
                armTargetPosition = Range.clip(armTargetPosition, ARM_MIN_POSITION, ARM_MAX_POSITION);
                armMotor.setTargetPosition(armTargetPosition);

                currentArmPower = calcArmPower();
                armMotor.setPower(currentArmPower);
                armMoveTriggered = true;
                lastArmMovementTime = System.currentTimeMillis();
                lastDPadUpPressTime = currentTime;
            //}
        }
        else {
            lastDPadUpPressTime = 0;
        }

        int extensionPosition = extensionArmMotor.getCurrentPosition();
        if (gamepad2.dpad_down && !predefinedActionRunning) {
            //if (currentTime - lastDPadDownPressTime > DEBOUNCE_DELAY) {
                double extensionFactor = (double) extensionPosition / EXTENSION_MAX_POSITION;
                // If the extension is fully extended (2200), set dynamicArmMinPosition to 200
                if (extensionPosition == EXTENSION_MAX_POSITION) {
                    dynamicArmMinPosition = 200;
                }
                // If the extension is at 900, set dynamicArmMinPosition to 100
                else if (extensionPosition == 900) {
                    dynamicArmMinPosition = 100;
                }
                // If the extension is fully retracted (0), set dynamicArmMinPosition to 100
                else if (extensionPosition == EXTENSION_MIN_POSITION) {
                    dynamicArmMinPosition = 100;
                }
                // If the extension is somewhere in between, calculate a value between 80 and 150
                else {
                    if (extensionPosition > 900) {
                        // Linearly interpolate between 900 and 2200 for the range [100, 200]
                        dynamicArmMinPosition = (int) (100 + (extensionPosition - 900) * (200 - 100) / (EXTENSION_MAX_POSITION - 900));
                    } else {
                        // Linearly interpolate between 0 and 900 for the range [100, 100]
                        dynamicArmMinPosition = (int) (100 + extensionPosition * (100 - 100) / 900);
                    }

                }

                long timeHeld = currentTime - lastDPadDownPressTime;

                //if (timeHeld < RAMP_TIME) {
                 //   armTargetPosition += Math.min((int) (ARM_MAX_SPEED * (timeHeld / (float) RAMP_TIME)), ARM_MAX_SPEED);
                //} else {
                    armTargetPosition -= ARM_MAX_SPEED;  // Max speed after ramp time
                //}
                armTargetPosition = Math.max(armTargetPosition, dynamicArmMinPosition);
                armTargetPosition = Range.clip(armTargetPosition, ARM_MIN_POSITION, ARM_MAX_POSITION);
                currentArmPower = calcArmPower();
                armMotor.setTargetPosition(armTargetPosition);
                armMotor.setPower(currentArmPower);

                armMoveTriggered = true;
                lastArmMovementTime = System.currentTimeMillis();
                lastDPadDownPressTime = currentTime;
            //}
        }
        else {
            lastDPadDownPressTime = 0;
        }

        // --- END ARM ROTATE MOTOR CONTROL ---

        // --- EXTENSION ARM MOTOR CONTROL ---

        if (gamepad2.x && !predefinedActionRunning) {

            long timeHeld = currentTime - lastXPressTime; // Time the X button has been held

            // Ramp speed over time if the button is held
            int rampSpeed = 0;
            if (timeHeld < EXTENSION_RAMP_TIME) {
                // Gradually increase the speed based on how long the button is held
                rampSpeed = (int) (EXTENSION_MAX_SPEED * (timeHeld / (float) EXTENSION_RAMP_TIME));
            } else {
                // Once RAMP_TIME has passed, use max speed
                rampSpeed = EXTENSION_MAX_SPEED;
            }

            // Update the target position based on the ramped speed
            extensionTargetPosition += rampSpeed;
            extensionTargetPosition = Math.min(extensionTargetPosition, EXTENSION_MAX_POSITION); // Clamp to max position

            // Optionally clamp to min position (to avoid retracting beyond the limit)
            extensionTargetPosition = Math.max(extensionTargetPosition, EXTENSION_MIN_POSITION);

            // Set motor target position and mode
            extensionArmMotor.setTargetPosition(extensionTargetPosition);
            extensionArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Calculate the motor power
            currentExtensionPower = calcExtensionPower();
            currentExtensionPower = Range.clip(extensionTargetPosition, EXTENSION_BASE_POWER, 1.0);

            // Set the motor power to control the speed (based on rampSpeed)
            extensionArmMotor.setPower(currentExtensionPower);

            // Update the last press time
            lastXPressTime = currentTime;
        } else {
            // If the button is not held, reset the last press time
            lastXPressTime = 0;
        }




        if (gamepad2.left_bumper && !predefinedActionRunning) {
            // Ramp speed over time if the button is held

            // Update the target position based on the ramped speed
            extensionTargetPosition += EXTENSION_PRESCION_SPEED;
            extensionTargetPosition = Math.min(extensionTargetPosition, EXTENSION_MAX_POSITION); // Clamp to max position

            // Calculate the motor power
            currentExtensionPower = calcExtensionPower();
            extensionArmMotor.setPower(currentExtensionPower);

        }


        if (gamepad2.right_bumper && !predefinedActionRunning) {
           // Update the target position based on the ramped speed
            extensionTargetPosition -= EXTENSION_PRESCION_SPEED;
            extensionTargetPosition = Math.min(extensionTargetPosition, EXTENSION_MAX_POSITION); // Clamp to max position

            // Optionally clamp to min position (to avoid retracting beyond the limit)
            extensionTargetPosition = Math.max(extensionTargetPosition, EXTENSION_MIN_POSITION);

            // Calculate the motor power
            currentExtensionPower = calcExtensionPower();
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

        if (gamepad2.y && !predefinedActionRunning) {
            //HOOK SET BUTTON
            // Get current positions
            long actionStartTime;
            int currentArmPosition = armMotor.getCurrentPosition();
            int currentExtensionPosition = extensionArmMotor.getCurrentPosition();
            predefinedActionRunning = true;
            if (currentArmPosition < HOOK_ARM_HEIGHT) {
                // Step 1: Retract the extension
                extensionTargetPosition = EXTENSION_MIN_POSITION;
                extensionArmMotor.setTargetPosition(extensionTargetPosition); // Fully retract
                extensionArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                currentExtensionPower = calcExtensionPower();
                extensionArmMotor.setPower(EXTENSION_BASE_POWER); // Set appropriate power
                actionStartTime = System.currentTimeMillis();
                while (extensionArmMotor.isBusy() && (System.currentTimeMillis() - actionStartTime) < HOOK_ACTION_TIMEOUT) {
                    if (Math.abs(extensionArmMotor.getCurrentPosition() - extensionTargetPosition) < MOTOR_TOLERANCE) {
                        break; // Break if within tolerance
                    }
                }

                // Step 2: Move the arm to the target height
                armTargetPosition = HOOK_ARM_HEIGHT;
                armMotor.setTargetPosition(armTargetPosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                currentArmPower = calcArmPower();
                armMotor.setPower(currentArmPower);
                actionStartTime = System.currentTimeMillis();
                while (armMotor.isBusy() && (System.currentTimeMillis() - actionStartTime) < HOOK_ACTION_TIMEOUT) {
                    if (Math.abs(armMotor.getCurrentPosition() - armTargetPosition) < MOTOR_TOLERANCE) {
                        break; // Break if within tolerance
                    }
                }


                // Step 3: Extend the extension to the target distance
                extensionTargetPosition = HOOK_EXTENSION_POSITION;
                extensionArmMotor.setTargetPosition(extensionTargetPosition);
                extensionArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                currentExtensionPower = calcExtensionPower();
                extensionArmMotor.setPower(currentExtensionPower);
                actionStartTime = System.currentTimeMillis();
                while (extensionArmMotor.isBusy() && (System.currentTimeMillis() - actionStartTime) < HOOK_ACTION_TIMEOUT) {
                    if (Math.abs(extensionArmMotor.getCurrentPosition() - extensionTargetPosition) < MOTOR_TOLERANCE) {
                        break; // Break if within tolerance
                    }
                }
            } else {
                // If the arm is already at or above the target height
                // Step 1: Move the arm to the target height
                armTargetPosition = HOOK_ARM_HEIGHT;
                armMotor.setTargetPosition(armTargetPosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                currentArmPower = calcArmPower();
                armMotor.setPower(currentArmPower);
                actionStartTime = System.currentTimeMillis();
                while (armMotor.isBusy() && (System.currentTimeMillis() - actionStartTime) < HOOK_ACTION_TIMEOUT) {
                    if (Math.abs(armMotor.getCurrentPosition() - armTargetPosition) < MOTOR_TOLERANCE) {
                        break; // Break if within tolerance
                    }
                }


                // Step 2: Extend the extension to the target distance
                extensionTargetPosition = HOOK_EXTENSION_POSITION;
                extensionArmMotor.setTargetPosition(extensionTargetPosition);
                extensionArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                currentExtensionPower = calcExtensionPower();
                extensionArmMotor.setPower(EXTENSION_BASE_POWER);
                actionStartTime = System.currentTimeMillis();
                while (extensionArmMotor.isBusy() && (System.currentTimeMillis() - actionStartTime) < HOOK_ACTION_TIMEOUT) {
                    if (Math.abs(extensionArmMotor.getCurrentPosition() - extensionTargetPosition) < MOTOR_TOLERANCE) {
                        break; // Break if within tolerance
                    }
                }
            }

            // Stop motors at the end to ensure no lingering movement
            armMotor.setPower(ARM_BASE_POWER);
            extensionArmMotor.setPower(0);
            predefinedActionRunning = false;
        }

        if (gamepad2.a && !predefinedActionRunning) {
            //HOOK RELEASE BUTTON -- ASSUMES CURRENTLY IN HOOK POSITION
            // Get current positions
            long actionStartTime;
            int currentArmPosition = armMotor.getCurrentPosition();
            int currentExtensionPosition = extensionArmMotor.getCurrentPosition();
            predefinedActionRunning = true;
            armTargetPosition = HOOK_RELEASE_ARM_HEIGHT;
            armMotor.setTargetPosition(armTargetPosition);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            currentArmPower = calcArmPower();
            armMotor.setPower(ARM_BASE_POWER);
            extensionTargetPosition = HOOK_RELEASE_EXTENSION_POSITION;
            extensionArmMotor.setTargetPosition(extensionTargetPosition);
            extensionArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            currentExtensionPower = calcExtensionPower();
            extensionArmMotor.setPower(currentExtensionPower);
            actionStartTime = System.currentTimeMillis();
            while ((armMotor.isBusy() || extensionArmMotor.isBusy()) && (System.currentTimeMillis() - actionStartTime) < HOOK_RELEASE_ACTION_TIMEOUT) {
                telemetry.addData("ACTION", "gamepad2.a");
                telemetry.addData("Arm Target", armTargetPosition);
                telemetry.addData("Arm Position", armMotor.getCurrentPosition());
                telemetry.addData("Extension Target", extensionTargetPosition);
                telemetry.addData("Extension Position", extensionArmMotor.getCurrentPosition());
                telemetry.update();

                if (Math.abs(armMotor.getCurrentPosition() - armTargetPosition) < MOTOR_TOLERANCE &&
                        Math.abs(extensionArmMotor.getCurrentPosition() - extensionTargetPosition) < MOTOR_TOLERANCE) {
                    break; // Break if within tolerance
                }
            }

            // Stop motors at the end to ensure no lingering movement
            armMotor.setPower(ARM_BASE_POWER);
            extensionArmMotor.setPower(0);
            predefinedActionRunning = false;
        }

        // --- END AUTOMATED MOVEMENT BUTTONS

        // --- STOP & EMERGENCY ACTIONS

        // Reset the arm position scale down the motor if it hasn't moved in 1 second (and it should be)
        if (armMoveTriggered && (currentTime - lastArmMovementTime) > 1000) {// && isArmStalled()) {
            armTargetPosition = armMotor.getCurrentPosition();
            armMotor.setTargetPosition(armTargetPosition);
            armMotor.setPower(calcArmPower());
            armMoveTriggered = false;  // Reset movement flag
            predefinedActionRunning = false;
        }

        //Stop the motor if it has reached the limit
        int currentPosition = extensionArmMotor.getCurrentPosition();
        if (currentPosition >= EXTENSION_MAX_POSITION || currentPosition <= EXTENSION_MIN_POSITION) {
            extensionArmMotor.setPower(0); // Stop the motor if it has reached the limit
        }


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

            predefinedActionRunning = false;
        }

        // --- END EMERGENCY ACTIONS

        // Send telemetry message to signify robot running;
        telemetry.addData("front",  "%.2f", front);
        telemetry.addData("turn", "%.2f", turn);
        telemetry.addData("Arm Motor Position", armMotor.getCurrentPosition());
        telemetry.addData("Arm Target Position", armTargetPosition);
        telemetry.addData("Arm Calculated Min Position", dynamicArmMinPosition);
        telemetry.addData("Arm Calculated Power", currentArmPower);
        telemetry.addData("Arm Motor Busy", armMotor.isBusy());
        telemetry.addData("Extension Current Position", extensionArmMotor.getCurrentPosition());
        telemetry.addData("Extension Target Position", extensionTargetPosition);
        //telemetry.addData("Extension Proximity Factor", extensionProximityFactor);
        telemetry.addData("Extension Calculated Power", currentExtensionPower);
        telemetry.addData("Extension Motor Busy", extensionArmMotor.isBusy());
        telemetry.addData("Left Servo Position", leftWheelServo.getPosition());
        telemetry.addData("Right Servo Position", rightWheelServo.getPosition());
        telemetry.addData("Running Defined Action", predefinedActionRunning);

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


    public double calcArmPower() {
        // Adjust arm motor power based on vertical arm position
        double horizontalArmPower = ARM_BASE_POWER;
        double verticalFactor = (double) extensionTargetPosition / EXTENSION_MAX_POSITION;
        horizontalArmPower += verticalFactor * ARM_EXTRA_FORCE; // Add extra power when fully raised
        horizontalArmPower = Range.clip(horizontalArmPower, 0.2, 1.0);
        return horizontalArmPower;
    }

    private boolean isArmStalled() {
        int currentPosition = armMotor.getCurrentPosition();
        return Math.abs(currentPosition - armTargetPosition) <= MOTOR_TOLERANCE;
    }
    public double getAngle() {
        double angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        return angle;
    }
}
    */
/*
     * Code to run ONCE after the driver hits STOP
     *//*


//    public void stop() {
//    }
//
//}
*/
