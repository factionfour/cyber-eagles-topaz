package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RobotPositionTracker {
    private final DcMotor leftDeadWheel;   // Left dead wheel motor
    private final DcMotor rightDeadWheel;  // Right dead wheel motor
    private IMU imu;                 // IMU for heading
    private final ElapsedTime runtime;     // Timer for elapsed time

    public double initXPosition;
    public double initYPosition;
    public double initAngle;

    // Robot position and heading
    private double xPositionCM = 0;        // X position in cm
    private double yPositionCM = 0;        // Y position in cm
    private double lastHeading = 0;        // Last recorded heading in radians

    // Conversion factors
    private static final double ENCODER_TICKS_PER_REV = 8192; // Example for Gobilda encoder
    private static final double WHEEL_DIAMETER_CM = 5.0;      // Diameter of the dead wheel in cm
    private static final double WHEEL_CIRCUMFERENCE_CM = Math.PI * WHEEL_DIAMETER_CM;
    private static final double TICKS_TO_CM = WHEEL_CIRCUMFERENCE_CM / ENCODER_TICKS_PER_REV;
    private static final double DESTINATION_TOLERANCE_CM = 2.0;  // tolerance for destination

    // Encoder deltas
    private int lastLeftEncoder = 0;
    private int lastRightEncoder = 0;

    //Robot movement destination
    double destHeading = 0;
    double destX = 0;
    double destY = 0;

    public RobotPositionTracker(HardwareMap hardwareMap) {
        // Initialize the dead wheel encoders
        leftDeadWheel = hardwareMap.get(DcMotor.class, "leftDeadWheel");
        rightDeadWheel = hardwareMap.get(DcMotor.class, "rightDeadWheel");

        // Reset and set encoders to RUN_WITHOUT_ENCODER mode
        leftDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDeadWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDeadWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize the IMU
        imu = hardwareMap.get(IMU.class, "imu");

        // Initialize timer
        runtime = new ElapsedTime();
    }

    // Update the robot's position
    public void updatePosition() {
        // Get current encoder positions
        int leftEncoder = leftDeadWheel.getCurrentPosition();
        int rightEncoder = rightDeadWheel.getCurrentPosition();

        // Get the current heading from the IMU
        double currentHeading = Math.toRadians(imu.getRobotYawPitchRollAngles().getYaw());

        // Calculate encoder deltas
        int deltaLeft = leftEncoder - lastLeftEncoder;
        int deltaRight = rightEncoder - lastRightEncoder;

        // Update last encoder positions
        lastLeftEncoder = leftEncoder;
        lastRightEncoder = rightEncoder;

        // Convert encoder ticks to distance in cm
        double distanceLeft = deltaLeft * TICKS_TO_CM;
        double distanceRight = deltaRight * TICKS_TO_CM;

        // Average distance traveled by both wheels (approximating center of robot)
        double distanceTraveled = (distanceLeft + distanceRight) / 2;

        // Calculate heading change
        double deltaHeading = currentHeading - lastHeading;

        // Normalize deltaHeading to [-pi, pi]
        if (deltaHeading > Math.PI) deltaHeading -= 2 * Math.PI;
        if (deltaHeading < -Math.PI) deltaHeading += 2 * Math.PI;

        // Update last heading
        lastHeading = currentHeading;

        // Calculate local X and Y displacement
        double deltaX = distanceTraveled * Math.cos(currentHeading);
        double deltaY = distanceTraveled * Math.sin(currentHeading);

        // Update global X and Y positions
        xPositionCM += deltaX;
        yPositionCM += deltaY;
    }

    // Get the current X position in cm
    public double getXPositionCM() {
        return xPositionCM;
    }

    // Get the current Y position in cm
    public double getYPositionCM() {
        return yPositionCM;
    }

    // Reset the robot's position
    public void resetPosition(double startXCM, double startYCM) {
        xPositionCM = startXCM;
        yPositionCM = startYCM;
        lastLeftEncoder = leftDeadWheel.getCurrentPosition();
        lastRightEncoder = rightDeadWheel.getCurrentPosition();
        lastHeading = Math.toRadians(imu.getRobotYawPitchRollAngles().getYaw());
    }

    // Set destination
    public void setDestination(double x, double y) {
        destX = x;
        destY = y;
    }

    public double getDistanceToDestination() {
        double deltaX = destX - xPositionCM;
        double deltaY = destY - yPositionCM;
        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }

    public boolean isDestinationReached() {
        return getDistanceToDestination() <= DESTINATION_TOLERANCE_CM;
    }
}
