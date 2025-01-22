
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
    public Servo Wrist = null;
//    public Servo Wrist;

    double DRIVING_SLOW =0.7;
    double DRIVING_SLOW_AUTO = 0.7;

    // Arm motor limits and power
    int ARM_MIN_POSITION =100;    // Minimum encoder position (fully Lowered// )
    int ARM_MAX_POSITION = 1300; // Maximum encoder position (fully Raised)
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

    double EXTENSION_MIN_SPEED = 0.3; // How far to move per iterationfsampleRelease
    double EXTENSION_MAX_SPEED = 0.7; // How far to move per iteration
    double EXTENSION_RAMP_TICKS = 50;
    double EXTENSION_MANUAL_MAX_SPEED = 100;

    //tolernances
    int MOTOR_TOLERANCE = 10; // Acceptable error in encoder ticks
    double POSITION_TOLERANCE_CM = 1.5;
    double POSITION_ONEDIRECTION_TOLERANCE_CM = 10;
    double HEADING_TOLERANCE_DEGREES = 3;

    //servo limits
    double SERVO_STOPPED = 0.5;
    double SERVO_FORWARD = 1;
    double SERVO_BACKWARD = 0;

    //PID turning variables
    private double Kp = 0.7; // Proportional gain
    private double Ki = 0.05; // Integral gain
    private double Kd = 0.2; // Derivative gain
    private double lastTurnError = 0; // Last error
    private double integralTurnError = 0; // Integral of the error
    private double lastPIDTime = 0; // Last time the PID calculation was done


    //pre-defined positions


  /*  int HOOK_RELEASE_EXTENSION_POSITION = 1400;
    int HOOK_RELEASE_ARM_HEIGHT = 670;
    int HOOK_RELEASE_RADIANS = 0;
    int HOOK_RELEASE_POS_X = 550;
    int HOOK_RELEASE_POS_Y = 200;

    int PICKUP_SPECIMEN_EXTENSION_POSITION = 600;
    int PICKUP_SPECIMEN_ARM_HEIGHT = 150;
    int PICKUP_SPECIMEN_RADIANS = 0;
    int PICKUP_SPECIMEN_POS_X = 0;
    int PICKUP_SPECIMEN_POS_Y = 0;
*/
    int HOOK_EXTENSION_POSITION = 0;//2100;
    int HOOK_ARM_HEIGHT =0; //750;
    int HOOK_DEGREES = 0;
    int HOOK_POS_X = 35;
    int HOOK_POS_Y = 35;

    int PICKUP_SAMPLE_ARM_HEIGHT =0;// 620;
    int PICKUP_SAMPLE_EXTENSION_POSITION =0;// 2013;
    int PICKUP_SAMPLE_DEGREES = -180;
    int PICKUP_SAMPLE_POS_X = 26;
    int PICKUP_SAMPLE_POS_Y = -30;//-98;

    int RELEASE_SAMPLE_ARM_HEIGHT = 0;//1040;
    int RELEASE_SAMPLE_EXTENSION_POSITION = 0;//2205;
    int RELEASE_SAMPLE_DEGREES = 0;//135;
    int RELEASE_SAMPLE_POS_X = 35;
    int RELEASE_SAMPLE_POS_Y = 17;//170;

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
    pickupSampleGroundState samplePickupState = pickupSampleGroundState.IDLE;
    releaseSampleFirstBucketState sampleReleaseState = releaseSampleFirstBucketState.IDLE;

    manualServoState tmpServoState = manualServoState.IDLE;
    driveToPositionState tmpDriveState = driveToPositionState.IDLE;
    wristServoState tempServoState = wristServoState.IDLE;


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
        Wrist = hardwareMap.get(Servo.class, "wrist");

        leftWheelServo.setPosition(0.5); // Neutral position
        rightWheelServo.setPosition(0.5); // Neutral position
        Wrist.setPosition(0.5);

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

        // --- STOP & EMERGENCY ACTIONS
        if (gamepad2.back) {
            emergencyReset();
        }

        if (gamepad1.y) {
            specimenHook();
        }
        else {
            specimenHookState = HookState.IDLE; // Transition to next step
        }

        if (gamepad1.x) {
            sampleRelease();
        }
        else {
            sampleReleaseState = releaseSampleFirstBucketState.IDLE; // Transition to next step
        }

        if (gamepad1.a) {
            samplePickupGround();
        }
        else {
            samplePickupState = samplePickupState.IDLE;
//            if (samplePickupState != pickupSampleGroundState.IDLE) {
//                moveIntake(false,false);
//                samplePickupState = samplePickupState.IDLE;
//            }
        }

        if (!isActionRunning()) {
            driveWheels(-gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, true);
//            moveIntake(gamepad2.left_bumper, gamepad2.right_bumper);
            moveArm(gamepad2.left_stick_y);
            moveExtension(gamepad2.right_stick_y);
            moveWrist(gamepad2.left_bumper,gamepad2.right_bumper);
        }
        addTelemetry();
    }



    public void addTelemetry() {
        telemetry.addData("Current X",  "%.2f", positionTracker.getXPositionCM());
        telemetry.addData("Current Y",  "%.2f", positionTracker.getYPositionCM());
        telemetry.addData("Current degrees", positionTracker.getHeadingDegrees());
        //telemetry.addData("front",  "%.2f", currentForward);
       //telemetry.addData("turn", "%.2f", currentTurn);
        //telemetry.addData("strafe", "%.2f", currentStrafe);
       // telemetry.addData("Arm Motor Position", armMotor.getCurrentPosition());
//telemetry.addData("Wrist servo",Wrist.getPosition());
//        telemetry.addData("Arm Target Position", armTargetPosition);
//        telemetry.addData("Arm Calculated Min Position", dynamicArmMinPosition);
//        telemetry.addData("Arm Calculated Power", currentArmPower);
//        telemetry.addData("Arm Motor Busy", armMotor.isBusy());
        //telemetry.addData("Extension Current Position", extensionArmMotor.getCurrentPosition());
//        telemetry.addData("Extension Target Position", extensionTargetPosition);
//        telemetry.addData("Extension Calculated Power", currentExtensionPower);
//        telemetry.addData("Extension Motor Busy", extensionArmMotor.isBusy());
//        telemetry.addData("Left Servo Position", leftWheelServo.getPosition());
//        telemetry.addData("Right Servo Position", rightWheelServo.getPosition());*/
        telemetry.update();
    }
//
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
        telemetry.addData("calc currentForward", currentForward);
        telemetry.addData("calc currentTurn", currentTurn);
        telemetry.addData("calc currentStrafe", currentStrafe);

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

        telemetry.addData("frontLeftPower", frontLeftPower);
        telemetry.addData("frontRightPower", frontRightPower);
        telemetry.addData("backLeftPower", backLeftPower);
        telemetry.addData("backRightPower", backRightPower);
        long currentTime = System.currentTimeMillis();
        positionTracker.updatePosition();
    }
//
//    public void driveWheels(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower, boolean human) {
//        // Normalize power values to avoid exceeding 1.0
//        double maxPower = Math.max(
//                Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
//                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
//        );
//
//        if (maxPower > 1.0) {
//            frontLeftPower /= maxPower;
//            frontRightPower /= maxPower;
//            backLeftPower /= maxPower;
//            backRightPower /= maxPower;
//        }
//
//        // Apply speed scaling for autonomous or human control
//        if (human) {
//            frontLeftPower *= DRIVING_SLOW;
//            frontRightPower *= DRIVING_SLOW;
//            backLeftPower *= DRIVING_SLOW;
//            backRightPower *= DRIVING_SLOW;
//        } else {
//            frontLeftPower *= DRIVING_SLOW_AUTO;
//            frontRightPower *= DRIVING_SLOW_AUTO;
//            backLeftPower *= DRIVING_SLOW_AUTO;
//            backRightPower *= DRIVING_SLOW_AUTO;
//        }
//
//        // Set motor powers
//        frontleftDrive.setPower(frontLeftPower);
//        frontrightDrive.setPower(frontRightPower);
//        backleftDrive.setPower(backLeftPower);
//        backrightDrive.setPower(backRightPower);
//
//        // Telemetry for debugging
//        telemetry.addData("Front Left Power", frontLeftPower);
//        telemetry.addData("Front Right Power", frontRightPower);
//        telemetry.addData("Back Left Power", backLeftPower);
//        telemetry.addData("Back Right Power", backRightPower);
//
//        // Update position tracker
//        long currentTime = System.currentTimeMillis();
//        positionTracker.updatePosition();
//    }

//
//    public boolean driveToPosition(double targetXCM, double targetYCM, double targetHeadingDegrees) {
//        if (tmpDriveState == driveToPositionState.IDLE) {
//            tmpDriveState = driveToPositionState.DRIVE;
//        }
//        // Convert targetHeading from degrees to radians
//        double targetHeadingRadians = Math.toRadians(targetHeadingDegrees);
//
//        // Update the robot's current position and heading
//        positionTracker.updatePosition();
//        double currentX = positionTracker.getXPositionCM();
//        double currentY = positionTracker.getYPositionCM();
//        double currentHeading = positionTracker.getHeading();
//        boolean atTargetPos = false;
//        boolean atTargetRad = false;
//        // Calculate differences to target
//        double deltaX = (targetXCM - currentX);
//        double deltaY = (targetYCM - currentY);
//        double distanceToTarget = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
//        double deltaHeading = targetHeadingRadians - currentHeading;
//
//        if (deltaHeading > Math.PI) deltaHeading -= 2 * Math.PI;
//        if (deltaHeading < -Math.PI) deltaHeading += 2 * Math.PI;
//
//        telemetry.addData("X TARGET",targetXCM);
//        telemetry.addData("Y TARGET", targetYCM);
//        telemetry.addData("Current Heading", Math.toDegrees(currentHeading));
//        telemetry.addData("Target Heading", targetHeadingDegrees);
//        telemetry.addData("delta X",deltaX);
//        telemetry.addData("delta Y", deltaY);
//        telemetry.addData("Delta Heading", Math.toDegrees(deltaHeading));
//        telemetry.addData("DRIVE STATE", tmpDriveState);
//        // Step 1: Move to the target position
//        if (distanceToTarget > POSITION_TOLERANCE_CM && tmpDriveState == driveToPositionState.DRIVE) {
//            double forwardPower = 0;
//            double strafePower = 0;
//            // Calculate power components for forward and strafe motion
////            double forwardPower = calculateDrivePower(deltaX); // Forward is X-axis
////            if (Math.abs(deltaX) < POSITION_TOLERANCE_CM) {
////                forwardPower = 0;
////            }
////            double strafePower = calculateDrivePower(deltaY);  // Strafe is Y-axis
////            if (Math.abs(deltaY) < POSITION_TOLERANCE_CM) {
////                strafePower = 0;
////            }
//
//            if (distanceToTarget > POSITION_ONEDIRECTION_TOLERANCE_CM) {
//                // Normal behavior: Apply both forward and strafe power
//                if (Math.abs(deltaX) > POSITION_TOLERANCE_CM) {
//                    forwardPower = calculateDrivePower(deltaX); // Forward is X-axis
//                }
//                if (Math.abs(deltaY) > POSITION_TOLERANCE_CM) {
//                    strafePower = calculateDrivePower(deltaY);  // Strafe is Y-axis
//                }
//            } else {
//                // Within 5 cm: Prioritize one direction at a time
//                if (Math.abs(deltaX) > POSITION_TOLERANCE_CM && Math.abs(deltaX) > Math.abs(deltaY)) {
//                    forwardPower = calculateDrivePower(deltaX); // Prioritize forward
//                } else if (Math.abs(deltaY) > POSITION_TOLERANCE_CM) {
//                    strafePower = calculateDrivePower(deltaY); // Prioritize strafe
//                }
//            }
//
//
//            // Normalize power values to ensure they don't exceed the max allowed
//            double maxPower = Math.max(Math.abs(forwardPower), Math.abs(strafePower));
//            if (maxPower > 1.0) {
//                forwardPower /= maxPower;
//                strafePower /= maxPower;
//            }
//            // Reverse directions if the robot has rotated more than 90 degrees
//            if (Math.abs(deltaHeading) > Math.PI / 2) { // More than 90 degrees
//                forwardPower = -forwardPower; // Reverse the forward direction
//                strafePower = -strafePower;   // Reverse the strafe direction
//            }
//            telemetry.addData("forwardPower", forwardPower);
//            telemetry.addData("strafePower", -strafePower);
//            // Drive the robot with both forward and strafe power
//            driveWheels(forwardPower, 0, -strafePower, false);
//            return false;
//        }
//        else {
//            atTargetPos = true;
//            if (tmpDriveState == driveToPositionState.DRIVE) {
//                tmpDriveState = driveToPositionState.TURN;
//                driveWheels(0, 0, 0,false);
//            }
//        }
//        atTargetRad = true;//DEBUG ONLY
//
////
////        // Step 2: Turn to the target heading
////        if (Math.abs(deltaHeading) > Math.toRadians(HEADING_TOLERANCE_DEGREES) && tmpDriveState == driveToPositionState.TURN) {
////            // Calculate turn power (proportional control)
////            double turnPower = calculateTurnPower(deltaHeading);
////        //if (Math.abs(deltaHeading) > Math.PI / 2) {
////         //   turnPower = -turnPower; // Reverse turn direction if heading change exceeds 90 degrees
////        //}
////            // Drive the robot with turn power
////            telemetry.addData("turnPower", -turnPower);
////            driveWheels(0, -turnPower, 0, false);
////        }
////        else {
////            atTargetRad = true;
////            if (tmpDriveState == driveToPositionState.TURN) {
////                tmpDriveState = driveToPositionState.COMPLETE;
////                driveWheels(0, 0, 0, false);
////            }
////        }
//        if (atTargetPos && atTargetRad) {
//            return true;
//        }
//        else {
//            return false;
//        }
//
//    }

    public boolean driveToPosition(double targetXCM, double targetYCM, double targetHeadingDegrees) {
        if (tmpDriveState == driveToPositionState.IDLE) {
            tmpDriveState = driveToPositionState.DRIVE;
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

        telemetry.addData("X TARGET", targetXCM);
        telemetry.addData("Y TARGET", targetYCM);
        telemetry.addData("Current Heading", Math.toDegrees(currentHeading));
        telemetry.addData("Target Heading", targetHeadingDegrees);
        telemetry.addData("Delta X", deltaX);
        telemetry.addData("Delta Y", deltaY);
        telemetry.addData("Delta Heading", Math.toDegrees(deltaHeading));
        telemetry.addData("DRIVE STATE", tmpDriveState);

        // Step 1: Move to the target position
        if (distanceToTarget > POSITION_TOLERANCE_CM && tmpDriveState == driveToPositionState.DRIVE) {
            // Calculate the angle to the target relative to the robot's position
            double angleToTarget = Math.atan2(deltaY, deltaX);
            double relativeAngleToTarget = angleToTarget - currentHeading;

            // Normalize the relative angle to the range [-π, π]
            if (relativeAngleToTarget > Math.PI) relativeAngleToTarget -= 2 * Math.PI;
            if (relativeAngleToTarget < -Math.PI) relativeAngleToTarget += 2 * Math.PI;

            // Calculate forward and strafe powers based on the relative angle
            double forwardPower = Math.cos(relativeAngleToTarget) * distanceToTarget;
            double strafePower = Math.sin(relativeAngleToTarget) * distanceToTarget;

            // Normalize power values to prevent exceeding max power
            double maxPower = Math.max(Math.abs(forwardPower), Math.abs(strafePower));
            if (maxPower > 1.0) {
                forwardPower /= maxPower;
                strafePower /= maxPower;
            }

            // Send adjusted power to the drive system
            driveWheels(forwardPower, 0, -strafePower, false);

            telemetry.addData("Forward Power", forwardPower);
            telemetry.addData("Strafe Power", -strafePower);
        } else {
            if (tmpDriveState == driveToPositionState.DRIVE) {

                tmpDriveState = driveToPositionState.TURN;
                driveWheels(0, 0, 0, false); // Stop movement
            }
        }

        // Step 2: Adjust heading if needed
        if (Math.abs(deltaHeading) > Math.toRadians(HEADING_TOLERANCE_DEGREES) && tmpDriveState == driveToPositionState.TURN) {
            double turnPower = calculateTurnPower(deltaHeading);

            // Ensure turnPower is applied in the correct direction
            turnPower *= Math.signum(deltaHeading);

            // Apply turn power
            driveWheels(0, turnPower, 0, false);

            telemetry.addData("Turn Power", turnPower);
        } else {
            if (tmpDriveState == driveToPositionState.TURN) {
                tmpDriveState = driveToPositionState.ADJUST;
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

            // Calculate forward and strafe powers based on the relative angle
            double forwardPower = Math.cos(relativeAngleToTarget) * distanceToTarget;
            double strafePower = Math.sin(relativeAngleToTarget) * distanceToTarget;

            // Normalize power values to prevent exceeding max power
            double maxPower = Math.max(Math.abs(forwardPower), Math.abs(strafePower));
            if (maxPower > 1.0) {
                forwardPower /= maxPower;
                strafePower /= maxPower;
            }

            // Send adjusted power to the drive system
            driveWheels(forwardPower, 0, -strafePower, false);

            telemetry.addData("Forward Power", forwardPower);
            telemetry.addData("Strafe Power", -strafePower);

        } else {
            if (tmpDriveState == driveToPositionState.ADJUST) {
                tmpDriveState = driveToPositionState.COMPLETE;
                driveWheels(0, 0, 0, false); // Stop movement
            }
        }

        // Return true if both position and heading are at the target
        return tmpDriveState == driveToPositionState.COMPLETE;
    }



//
//    public boolean driveToPosition(double targetXCM, double targetYCM, double targetHeadingDegrees) {
//        if (tmpDriveState == driveToPositionState.IDLE) {
//            tmpDriveState = driveToPositionState.DRIVE;
//        }
//
//        boolean atTargetPos = false;
//        boolean atTargetRad = false;
//
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
//        // Normalize deltaHeading to the range [-π, π]
//        if (deltaHeading > Math.PI) deltaHeading -= 2 * Math.PI;
//        if (deltaHeading < -Math.PI) deltaHeading += 2 * Math.PI;
//
//        telemetry.addData("X TARGET", targetXCM);
//        telemetry.addData("Y TARGET", targetYCM);
//        telemetry.addData("Current Heading", Math.toDegrees(currentHeading));
//        telemetry.addData("Target Heading", targetHeadingDegrees);
//        telemetry.addData("Delta X", deltaX);
//        telemetry.addData("Delta Y", deltaY);
//        telemetry.addData("Delta Heading", Math.toDegrees(deltaHeading));
//        telemetry.addData("DRIVE STATE", tmpDriveState);
//
//        // Step 1: Move to the target position
//        if (distanceToTarget > POSITION_TOLERANCE_CM && tmpDriveState == driveToPositionState.DRIVE) {
//            double forwardPower = 0.0;
//            double strafePower = 0.0;
//
//            // If within 10 cm of the target, prioritize one axis at a time
//            if (Math.abs(deltaX) <= 10.0 && Math.abs(deltaY) <= POSITION_ONEDIRECTION_TOLERANCE_CM) {
//                if (Math.abs(deltaX) > Math.abs(deltaY)) {
//                    // Prioritize X movement
//                    forwardPower = calculateDrivePower(deltaX);
//                } else {
//                    // Prioritize Y movement
//                    strafePower = calculateDrivePower(deltaY);
//                }
//            } else {
//                // Calculate power components for both axes
//                forwardPower = calculateDrivePower(deltaX); // Forward is X-axis
//                strafePower = calculateDrivePower(deltaY);  // Strafe is Y-axis
//            }
//
//            // Reverse directions based on the target position, not deltaHeading
//            if (deltaX < 0) {
//                forwardPower = -forwardPower; // Reverse forward direction if target X is to the left
//            }
//            if (deltaY < 0) {
//                strafePower = -strafePower; // Reverse strafe direction if target Y is behind
//            }
//
//            // Normalize power values to prevent exceeding max power
//            double maxPower = Math.max(Math.abs(forwardPower), Math.abs(strafePower));
//            if (maxPower > 1.0) {
//                forwardPower /= maxPower;
//                strafePower /= maxPower;
//            }
//
//            // Send power to the drive system
//            driveWheels(forwardPower, 0, -strafePower, false);
//
//            telemetry.addData("Forward Power", forwardPower);
//            telemetry.addData("Strafe Power", -strafePower);
//
//            return false;
//        } else {
//            if (tmpDriveState == driveToPositionState.DRIVE) {
//                atTargetPos = true;
//                tmpDriveState = driveToPositionState.TURN;
//                driveWheels(0, 0, 0, false); // Stop movement
//            }
//        }
//
//        atTargetRad = true;
////        // Step 2: Adjust heading if needed
////        if (Math.abs(deltaHeading) > Math.toRadians(HEADING_TOLERANCE_DEGREES) && tmpDriveState == driveToPositionState.TURN) {
////            double turnPower = calculateTurnPower(deltaHeading);
////
////            // Apply turn power
////            driveWheels(0, turnPower, 0, false);
////
////            telemetry.addData("Turn Power", turnPower);
////            return false;
////        } else {
////            if (tmpDriveState == driveToPositionState.TURN) {
////                atTargetRad = true;
////                tmpDriveState = driveToPositionState.COMPLETE;
////                driveWheels(0, 0, 0, false); // Stop turning
////            }
////        }
//
//        if (atTargetPos && atTargetRad) {
//            return true;
//        } else {
//            return false;
//        }
//    }


    private double calculateDrivePower(double distance) {
        double maxPower = 0.7; // Maximum power
        double minPower = 0.15; // Minimum power for precision (can be adjusted)
        double kDrive = 0.5;  // Proportional control factor
        double slowDownThreshold = 10.0; // Distance in cm where slowdown begins
        double stopThreshold = 2.0; // Distance in cm where it should stop but still move

        // If the robot is within the stop threshold, apply a minimum power to move slowly
        if (Math.abs(distance) <= stopThreshold) {
            return Math.signum(distance) * minPower; // Move slowly in the correct direction
        }

        // Apply proportional control: slower as the robot approaches the target
        double power = Math.abs(distance) > slowDownThreshold
                ? maxPower // Full speed for larger distances
                : Math.max(minPower, Math.min(maxPower, Math.abs(distance) * kDrive));

        // Normalize power to avoid exceeding max or min power
        return (distance < 0) ? -power : power;
    }

    private double calculateTurnPower(double deltaHeading) {
        double maxTurnPower = 0.7; // Maximum turning power
        double minTurnPower = 0.2; // Minimum turning power for precision
        double slowDownThreshold = Math.toRadians(20.0); // Angle threshold for starting to slow down
        double stopThreshold = Math.toRadians(HEADING_TOLERANCE_DEGREES); // Final threshold to stop turning

        // Normalize deltaHeading to the range [-π, π] to avoid large jumps across 180 degrees
        if (deltaHeading > Math.PI) {
            deltaHeading -= 2 * Math.PI; // Normalize to [-π, π]
        } else if (deltaHeading < -Math.PI) {
            deltaHeading += 2 * Math.PI;
        }

        // If within the final tolerance range, stop turning
        if (Math.abs(deltaHeading) <= stopThreshold) {
            telemetry.addData("TURN ","STOP");
            return 0.0; // Stop turning when within tolerance range
        }

        // If the robot is within the slow down threshold, apply the slow turn power
        if (Math.abs(deltaHeading) <= slowDownThreshold) {
            telemetry.addData("TURN ","SLOW");
            return minTurnPower * Math.signum(deltaHeading); // Slow down the turn
        }
        telemetry.addData("TURN ","FULL");
        // Apply full turn power for larger heading differences
        return maxTurnPower * Math.signum(deltaHeading); // Apply max turn power
    }






//    // Helper method to calculate turn power based on heading difference
//    private double calculateTurnPower(double deltaHeading) {
//        double maxTurnPower = 0.7; // Maximum turning power
//        double minTurnPower = 0.2; // Minimum turning power for precision
//        double kTurn = 0.15;  // Proportional control factor for turning
//        double slowDownThreshold = 5.0; // Angle threshold for slowing down turning
//
//        // Normalize deltaHeading to the range [-π, π] to avoid large jumps across 180 degrees
//        if (deltaHeading > Math.PI) {
//            deltaHeading -= 2 * Math.PI; // Normalize to [-π, π]
//        } else if (deltaHeading < -Math.PI) {
//            deltaHeading += 2 * Math.PI;
//        }
//
//        double headingTolerance = Math.toRadians(HEADING_TOLERANCE_DEGREES);
//        // Apply a tolerance threshold for when we are close enough to the target heading
//        if (Math.abs(deltaHeading) <= headingTolerance) {
//            return 0.0; // Stop turning when within tolerance range
//        }
//
//        // If the heading difference is very small, apply a minimum turn power
//        if (Math.abs(deltaHeading) <= slowDownThreshold) {
//            //return Math.signum(deltaHeading) * minTurnPower; // Slow turn in the correct direction
//            return -minTurnPower * Math.signum(deltaHeading); // Reverse turn direction
//        }
//
//        // Apply proportional control: the larger the difference, the more power is applied
//        double power = Math.abs(deltaHeading) * kTurn;
//
//        // Clamp power to the maximum allowed turning power
//        if (power > maxTurnPower) {
//            power = maxTurnPower;
//        }
//
//        // Reverse the direction of the turn if needed
//        // Flip sign here if the turn is not in the correct direction
//        return -power * Math.signum(deltaHeading); // Reverse turn direction
//    }

/*
    // Method to calculate turn power using PID
    private double calculateTurnPower(double deltaHeading) {
        // Calculate the error
        double error = deltaHeading;
        long currentTime = System.currentTimeMillis();
        double deltaTime = (currentTime - lastPIDTime) / 1000.0; // Time difference in seconds

        // Update integral (sum of past errors) and derivative (rate of change of error)
        integralTurnError += error * deltaTime;
        double derivative = (error - lastTurnError) / deltaTime;

        // Calculate the PID output
        double pidOutput = Kp * error + Ki * integralTurnError + Kd * derivative;

        // Update last error and last time
        lastTurnError = error;
        lastPIDTime = currentTime;

        // Clamp the output to a range, typically -1 to 1
        pidOutput = Math.max(-1.0, Math.min(1.0, pidOutput));

        return pidOutput;
    }*/


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

    //raise or lower the robot's arm to a specific height
    public Boolean moveArmEncoder(int startPosition, int targetPosition) {
        Boolean complete = false;
        int currentPos = armMotor.getCurrentPosition();
        int distanceToTarget = Math.abs(targetPosition - currentPos);
        int distanceFromStart = Math.abs(currentPos - startPosition);
        double currentSpeed = 0;
        if (currentPos >= (targetPosition - MOTOR_TOLERANCE) && currentPos <= (targetPosition + MOTOR_TOLERANCE)) {
            complete = true;
            armMotor.setPower(0);
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
        currentArmPower = calcArmPower();
        armMotor.setPower(currentArmPower);

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

    private void specimenHook() {
        telemetry.addData("CURRENT ACTION STATE",specimenHookState);
        //SPECIMEN HOOK
        if (specimenHookState == HookState.IDLE) {
            specimenHookState = HookState.POSITION_ROBOT; // Start first step
            tmpExtensionPositionHolder = extensionArmMotor.getCurrentPosition();
            tmpArmPositionHolder = armMotor.getCurrentPosition();
        }

        // Execute multi-step process based on current state
        switch (specimenHookState) {
            case POSITION_ROBOT:
                if (driveToPosition(HOOK_POS_X,HOOK_POS_Y,HOOK_DEGREES)) {
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
                moveIntake(false, true);
                if (elapsedTime > 500) {
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

    private void sampleRelease() {
        telemetry.addData("CURRENT ACTION STATE",sampleReleaseState);
        //SAMPLE RELEASE TO FIRST BUCKET
        if (sampleReleaseState == releaseSampleFirstBucketState.IDLE) {
            sampleReleaseState = releaseSampleFirstBucketState.POSITION_ROBOT; // Start first step
            tmpExtensionPositionHolder = extensionArmMotor.getCurrentPosition();
            tmpArmPositionHolder = armMotor.getCurrentPosition();
        }

        // Execute multi-step process based on current state
        switch (sampleReleaseState) {
            case POSITION_ROBOT:
                if (driveToPosition(RELEASE_SAMPLE_POS_X,RELEASE_SAMPLE_POS_Y,RELEASE_SAMPLE_DEGREES)) {
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
        telemetry.addData("CURRENT ACTION STATE",samplePickupState);
        //SAMPLE PICKUP FROM GROUND
        if (samplePickupState == pickupSampleGroundState.IDLE.IDLE) {
            samplePickupState = pickupSampleGroundState.POSITION_ROBOT; // Start first step
            tmpExtensionPositionHolder = extensionArmMotor.getCurrentPosition();
            tmpArmPositionHolder = armMotor.getCurrentPosition();
        }

        // Execute multi-step process based on current state
        switch (samplePickupState) {
            case POSITION_ROBOT:
                if (driveToPosition(PICKUP_SAMPLE_POS_X,PICKUP_SAMPLE_POS_Y,PICKUP_SAMPLE_DEGREES)) {
                    samplePickupState = pickupSampleGroundState.MOVE_ARM; // Transition to next step
                }
                break;
            case MOVE_ARM:
                if (moveArmEncoder(tmpExtensionPositionHolder,PICKUP_SAMPLE_ARM_HEIGHT) && moveExtensionEncoder(tmpExtensionPositionHolder,0)) { // && moveWrist) {
                    tmpActionStartTime = System.currentTimeMillis();
                    samplePickupState = pickupSampleGroundState.INTAKE; // Transition to next step
                }
                break;
            case INTAKE:
                long elapsedTime = System.currentTimeMillis() - tmpActionStartTime;
                telemetry.addData("INTAKE", "Elapsed Time: " + elapsedTime + " ms");
                // Move the intake motor
                moveIntake(true, false);
                // Check if the time limit has been exceeded or if the button was pressed
                if (elapsedTime > 6000) {
                    telemetry.addData("INTAKE", "6 seconds elapsed, moving to COMPLETE");
                    samplePickupState = pickupSampleGroundState.COMPLETE; // Transition to next step
                }
                break;
            case COMPLETE:

                setDefaultPower();
                break;
        }
    }

    private boolean isActionRunning() {
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
        INTAKE,
        COMPLETE
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

