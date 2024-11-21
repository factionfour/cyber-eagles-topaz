
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Robot: Teleop Tank", group="Robot")

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

    // Arm motor limits and power
    int armTargetPosition = 0;
    int ARM_MIN_POSITION = 0;    // Minimum encoder position (fully retracted)
    int ARM_MAX_POSITION = 450; // Maximum encoder position (fully extended)
    double ARM_BASE_POWER = 0.5;
    double ARM_EXTRA_FORCE = 0.6;


// Vertical arm motor limits and base power
    int extensionTargetPosition = 0;
    int EXTENSION_MIN_POSITION = 0;    // Minimum height (fully lowered)
    int EXTENSION_MAX_POSITION = 2200; // Maximum height (fully raised)
    double EXTENSION_BASE_POWER = 0.4; // Base power to hold position
    double EXTENSION_EXTRA_FORCE = 0.6; // Extra power when arm is extended
    int EXTENSION_MOVEMENT_PERTICK = 15; // How far to move per loop iteration

    //servo limits
    double SERVO_STOPPED = 0.5;
    double SERVO_FORWARD = 1;
    double SERVO_BACKWARD = 0;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    //debounce variables
    boolean wasXPressed = false;
    boolean wasBPressed = false;
    long lastExtensionMovementTime = System.currentTimeMillis(); // Last time the arm moved
    boolean extensionMoveTriggered = false; // Flag to indicate if the arm moved

    long lastArmMovementTime = System.currentTimeMillis(); // Last time the arm moved
    boolean armMoveTriggered = false; // Flag to indicate if the arm moved
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
        armMotor.setPower(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Initialize extension arm motor
        extensionArmMotor = hardwareMap.get(DcMotor.class, "arm_extendo");
        extensionArmMotor.setDirection(DcMotor.Direction.REVERSE);
        extensionArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extensionArmMotor.setTargetPosition(0);
        extensionArmMotor.setPower(0);
        extensionArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

/*        extensionArmMotor.setPower(1.0); // Full power to move to position quickly

        // Wait until the arm reaches the fully retracted position
        while (extensionArmMotor.isBusy()) {
            // Optionally add telemetry here for debugging
            telemetry.addData("Resetting Arm Position", "Moving to Zero Position");
            telemetry.update();
        }
        // Once done, stop the motor
        extensionArmMotor.setPower(0);*/

        // Initialize wheel servos
        leftWheelServo = hardwareMap.get(Servo.class, "servo_one");
        rightWheelServo = hardwareMap.get(Servo.class, "servo_two");
        leftWheelServo.setPosition(0.5); // Neutral position
        rightWheelServo.setPosition(0.5); // Neutral position



        telemetry.addData(">", "Robot Ready.  Press START.");    //
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
        double front;
        double turn;
        double strafe;

        // Run wheels in tank mode (note: The joystick goes negative when pushed forward, so negate it)
        front = gamepad1.left_stick_y;
        turn = gamepad1.right_stick_x;
        strafe = gamepad1.left_stick_x;

        frontleftDrive.setPower(-front);
        frontrightDrive.setPower(-front);
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

        // --- Arm Rotate Motor Control ---
        if (gamepad2.dpad_up) {
            armTargetPosition = Math.min(armTargetPosition + 10, ARM_MAX_POSITION);
            armMoveTriggered = true;
            lastArmMovementTime = System.currentTimeMillis();
        }
        if (gamepad2.dpad_down) {
            armTargetPosition = Math.max(armTargetPosition - 10, ARM_MIN_POSITION);
            armMoveTriggered = true;
            lastArmMovementTime = System.currentTimeMillis();
        }

        // Adjust arm motor power based on vertical arm position
        double horizontalArmPower = ARM_BASE_POWER;

        double verticalFactor = (double) extensionTargetPosition / EXTENSION_MAX_POSITION;
        horizontalArmPower += verticalFactor * ARM_EXTRA_FORCE; // Add extra power when fully raised
        long currentTime = System.currentTimeMillis();
        // Reset the arm position scale down the motor if it hasn't moved in .5 second
        if (armMoveTriggered && (currentTime - lastArmMovementTime) > 500) {
            armTargetPosition = armMotor.getCurrentPosition();
            armMotor.setTargetPosition(armTargetPosition);
            //horizontalArmPower = ARM_BASE_POWER;
            //armMotor.setPower(horizontalArmPower);
            armMoveTriggered = false;  // Reset movement flag
        }
        else {
            //Normal operations
            armMotor.setTargetPosition(armTargetPosition);
            armMotor.setPower(horizontalArmPower);
        }
        /*if (gamepad2.x) {
            extensionTargetPosition = Math.max(extensionTargetPosition - EXTENSION_MOVEMENT_PERTICK, EXTENSION_MIN_POSITION);
            extensionArmMotor.setTargetPosition(extensionTargetPosition);
            extensionArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            extensionArmMotor.setPower(0.5); // Set constant power for testing
        }

        if (gamepad2.b) {
            extensionTargetPosition = Math.min(extensionTargetPosition + EXTENSION_MOVEMENT_PERTICK, EXTENSION_MAX_POSITION);
            extensionArmMotor.setTargetPosition(extensionTargetPosition);
            extensionArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            extensionArmMotor.setPower(0.5); // Set constant power for testing
        }*/
        // --- Extension Arm Motor Control ---
        if (gamepad2.x) {
            extensionTargetPosition = Math.min(extensionTargetPosition + EXTENSION_MOVEMENT_PERTICK, EXTENSION_MAX_POSITION); // Ensure it doesn't go beyond max
            extensionMoveTriggered = true;
            lastExtensionMovementTime = System.currentTimeMillis();
        }

        if (gamepad2.b) {
            extensionTargetPosition = Math.max(extensionTargetPosition - EXTENSION_MOVEMENT_PERTICK, EXTENSION_MIN_POSITION); // Ensure it doesn't go below min
            extensionMoveTriggered = true;
            lastExtensionMovementTime = System.currentTimeMillis();
        }

        // Clamp target position to stay within bounds
        extensionTargetPosition = Range.clip(extensionTargetPosition, EXTENSION_MIN_POSITION, EXTENSION_MAX_POSITION);

        extensionArmMotor.setTargetPosition(extensionTargetPosition);
        extensionArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

        currentTime = System.currentTimeMillis();
        // Reset the position and turn off motor if it hasn't moved in 1 second
        if (extensionMoveTriggered && (currentTime - lastExtensionMovementTime) > 1000) {
            extensionTargetPosition = extensionArmMotor.getCurrentPosition();
            extensionArmMotor.setTargetPosition(extensionTargetPosition);
            extensionArmMotor.setPower(0);
            extensionMoveTriggered = false;  // Reset movement flag
            // Check if the target position is zero and motor hasn't moved for 1 second
            if (extensionTargetPosition == 0 && Math.abs(extensionArmMotor.getCurrentPosition()) < 10) {
                // Force the encoder to zero if motor hasn't moved
                extensionArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the encoder
                extensionArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Set back to run mode
                telemetry.addData("Extension Reset", "Encoder force-set to zero");
                telemetry.update();
            }
        }
        else {
            //normal operations
            extensionArmMotor.setPower(extensionPower);
            //Stop the motor if it has reached the limit
            if (currentPosition >= EXTENSION_MAX_POSITION || currentPosition <= EXTENSION_MIN_POSITION) {
                extensionArmMotor.setPower(0); // Stop the motor if it has reached the limit
            }
        }

        // --- Wheel Servo Control ---
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

        // Send telemetry message to signify robot running;
        telemetry.addData("front",  "%.2f", front);
        telemetry.addData("turn", "%.2f", turn);
        telemetry.addData("Arm Motor Position", armMotor.getCurrentPosition());
        telemetry.addData("Arm Target Position", armTargetPosition);
        telemetry.addData("Arm Calculated Power", horizontalArmPower);
        telemetry.addData("Arm Motor Busy", armMotor.isBusy());

        telemetry.addData("Extension Current Position", extensionArmMotor.getCurrentPosition());
        telemetry.addData("Extension Target Position", extensionTargetPosition);
        telemetry.addData("Extension Proximity Factor", extensionProximityFactor);
        telemetry.addData("Extension Calculated Power", extensionPower);
        telemetry.addData("Extension Motor Busy", extensionArmMotor.isBusy());
        telemetry.addData("Left Servo Position", leftWheelServo.getPosition());
        telemetry.addData("Right Servo Position", rightWheelServo.getPosition());
    }
}
    /*
     * Code to run ONCE after the driver hits STOP
     */

//    public void stop() {
//    }
//
//}
