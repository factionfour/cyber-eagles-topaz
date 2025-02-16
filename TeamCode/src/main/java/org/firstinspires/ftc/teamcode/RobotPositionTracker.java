package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import android.content.SharedPreferences;
import android.preference.PreferenceManager;


public class RobotPositionTracker {
    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    //private final DcMotor leftDeadWheel;   // Left dead wheel motor
    //private final DcMotor rightDeadWheel;  // Right dead wheel motor
    private IMU imu;                 // IMU for heading
    //private final ElapsedTime runtime;     // Timer for elapsed time

    public double initHeading = 0;
    public double initHeadingDegrees = 0;
    public double currentHeading = 0;
    public double currentHeadingDegrees = 0;

    // Robot position and heading
    public double currentPositionXCM = 0;        // X position in cm
    public double currentPositionYCM = 0;        // Y position in cm
    public double initXPositionCM;
    public double initYPositionCM;
//
//    // Conversion factors
//    private static final double ENCODER_TICKS_PER_REV = 8192; // Example for Gobilda encoder
//    private static final double WHEEL_DIAMETER_CM = 5.0;      // Diameter of the dead wheel in cm
//    private static final double WHEEL_CIRCUMFERENCE_CM = Math.PI * WHEEL_DIAMETER_CM;
//    private static final double TICKS_TO_CM = WHEEL_CIRCUMFERENCE_CM / ENCODER_TICKS_PER_REV;
//    private static final double DESTINATION_TOLERANCE_CM = 2.0;  // tolerance for destination

    //Robot movement destination  -- NOT YET USED
    double destHeading = 0;
    double destX = 0;
    double destY = 0;

    public RobotPositionTracker(GoBildaPinpointDriver myOdo, IMU myImu) {
        // Initialize the dead wheel encoders
        odo = myOdo;
        //TODO:UPDATE THESE
        odo.setOffsets(-13, 0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        // Initialize the IMU
        imu = myImu;
        odo.recalibrateIMU();
        Pose2D pos = odo.getPosition();

        initHeading = pos.getHeading(AngleUnit.RADIANS);
        initHeadingDegrees = pos.getHeading(AngleUnit.DEGREES);
        initXPositionCM = pos.getX(DistanceUnit.CM);
        initYPositionCM = pos.getY(DistanceUnit.CM);

        //runtime = new ElapsedTime();
    }

    // Update the robot's position
    public void updatePosition() {
        odo.update();
        Pose2D pos = odo.getPosition();

        // Get the current heading from the IMU
        currentHeading =  pos.getHeading(AngleUnit.RADIANS);
        currentHeadingDegrees =  pos.getHeading(AngleUnit.DEGREES);

        // Update last encoder positions

        currentPositionXCM = pos.getX(DistanceUnit.CM);
        currentPositionYCM = pos.getY(DistanceUnit.CM);

    }

    // Get the current X position in cm
    public double getXPositionCM() {
        return currentPositionXCM;
    }

    // Get the current Y position in cm
    public double getYPositionCM() {
        return currentPositionYCM;
    }

    public double getHeading() {
        return currentHeading;
    }

    public double getHeadingDegrees() {
        return currentHeadingDegrees;
    }

    public boolean resetPositionCheck(double startXCM, double startYCM, double startRadians) {
        initXPositionCM = startXCM;
        initYPositionCM = startYCM;
        //currentPositionXCM = startXCM;
        //currentPositionYCM = startYCM;
        Pose2D pos = new Pose2D(DistanceUnit.CM,startXCM,startYCM,AngleUnit.RADIANS,startRadians);
        odo.setPosition(pos);
        boolean isCorrect = false;
        updatePosition();
        if (startXCM == currentPositionXCM && startYCM == currentPositionYCM && startRadians == currentHeading) {
            isCorrect = true;
        }

        return isCorrect;
    }

    public void resetPosition(double startXCM, double startYCM, double startRadians) {
        initXPositionCM = startXCM;
        initYPositionCM = startYCM;
        currentPositionXCM = startXCM;
        currentPositionYCM = startYCM;
        Pose2D pos = new Pose2D(DistanceUnit.CM,startXCM,startYCM,AngleUnit.RADIANS,startRadians);
        odo.setPosition(pos);
    }


    public void saveRobotPosition(android.content.Context appContext) {
        SharedPreferences prefs = appContext.getSharedPreferences("RobotPrefs", appContext.MODE_PRIVATE);
        SharedPreferences.Editor editor = prefs.edit();
        editor.putFloat("robot_x", (float) currentPositionXCM);
        editor.putFloat("robot_y", (float) currentPositionYCM);
        editor.putFloat("robot_heading", (float) currentHeading);
        editor.apply(); // Commit changes asynchronously
    }

    public void loadRobotPosition(android.content.Context appContext) {
        SharedPreferences prefs = appContext.getSharedPreferences("RobotPrefs", appContext.MODE_PRIVATE);
        double x = prefs.getFloat("robot_x", 0); // Default to 0 if not set
        double y = prefs.getFloat("robot_y", 0);
        double heading = prefs.getFloat("robot_heading", 0);

        resetPosition(x,y,heading);

    }
}