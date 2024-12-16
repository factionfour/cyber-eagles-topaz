package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class RobotPositionTracker {
    // Constants
    private static final double WHEEL_CIRCUMFERENCE_CM = 10.0; // Circumference of the dead wheels in cm
    private static final int ENCODER_COUNTS_PER_REVOLUTION = 1440; // Encoder counts per wheel revolution
    private static final double ENCODER_TO_CM = WHEEL_CIRCUMFERENCE_CM / ENCODER_COUNTS_PER_REVOLUTION;

    // Position tracking variables
    private double xPosition = 0.0; // Current X position in cm
    private double yPosition = 0.0; // Current Y position in cm
    private double headingRadians = 0.0; // Current heading in radians

    // Encoder tracking
    private double previousLeftEncoder = 0.0;
    private double previousRightEncoder = 0.0;

    // Hardware
    private Encoder leftDeadWheel;   // Left dead wheel encoder
    private Encoder rightDeadWheel;  // Right dead wheel encoder
    private IMU imu;                 // IMU for robot heading

    // Timer for telemetry updates
    private ElapsedTime runtime;

    // Constructor
    public RobotPositionTracker(HardwareMap hardwareMap) {
        // Initialize hardware
        leftDeadWheel = new Encoder(hardwareMap.get(DcMotor.class, "leftDeadWheel"));
        rightDeadWheel = new Encoder(hardwareMap.get(DcMotor.class, "rightDeadWheel"));
        imu = hardwareMap.get(IMU.class, "imu");
        runtime = new ElapsedTime();
    }

    // Reset the robot's position and heading
    public void resetPosition(double startX, double startY, double startHeadingDegrees) {
        xPosition = startX;
        yPosition = startY;
        headingRadians = Math.toRadians(startHeadingDegrees);

        // Reset encoders to the current position
        previousLeftEncoder = leftDeadWheel.getCurrentPosition();
        previousRightEncoder = rightDeadWheel.getCurrentPosition();
    }

    // Update the robot's position
    public void updatePosition() {
        // Get the current encoder values
        double currentLeftEncoder = leftDeadWheel.getCurrentPosition();
        double currentRightEncoder = rightDeadWheel.getCurrentPosition();

        // Calculate the distance traveled since the last update
        double leftDeltaCM = (currentLeftEncoder - previousLeftEncoder) * ENCODER_TO_CM;
        double rightDeltaCM = (currentRightEncoder - previousRightEncoder) * ENCODER_TO_CM;

        // Update the previous encoder positions
        previousLeftEncoder = currentLeftEncoder;
        previousRightEncoder = currentRightEncoder;

        // Get the robot's current heading from the IMU
        headingRadians = Math.toRadians(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        // Compute the average forward movement (along the Y-axis of the robot)
        double forwardDeltaCM = (leftDeltaCM + rightDeltaCM) / 2.0;

        // Calculate the change in X and Y position in the arena
        double deltaX = forwardDeltaCM * Math.sin(headingRadians); // Lateral movement
        double deltaY = forwardDeltaCM * Math.cos(headingRadians); // Forward movement

        // Update the robot's global position
        xPosition += deltaX;
        yPosition += deltaY;
    }

    // Get the robot's current X position in cm
    public double getXPosition() {
        return xPosition;
    }

    // Get the robot's current Y position in cm
    public double getYPosition() {
        return yPosition;
    }

    // Get the robot's current heading in degrees
    public double getHeadingDegrees() {
        return Math.toDegrees(headingRadians);
    }

    // Output telemetry (optional for debugging)
    public void displayTelemetry(Telemetry telemetry) {
        telemetry.addData("X Position (cm)", xPosition);
        telemetry.addData("Y Position (cm)", yPosition);
        telemetry.addData("Heading (degrees)", Math.toDegrees(headingRadians));
        telemetry.update();
    }
}
