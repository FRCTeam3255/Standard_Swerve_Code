package frc.robot;

import com.frcteam3255.preferences.SN_DoublePreference;

import edu.wpi.first.units.Units;

public class RobotPreferences {
  public static final class prefDrivetrain {
    public static final double minimumSteerSpeedPercent = 0.01;

    // Translational speed (feet per second) while manually driving
    public static final double driveSpeed = Constants.constDrivetrain.DRIVE_SPEED.in(Units.MetersPerSecond);

    // Rotational speed (degrees per second) while manually driving
    public static final double turnSpeed = 360;

    public static final double autoMaxSpeedFeet = 8;
    public static final double autoMaxAccelFeet = 6;

    /**
     * <p>
     * Pose estimator standard deviation for encoder & gyro data
     * </p>
     * <b>Units:</b> Meters
     */
    public static final double measurementStdDevsPosition = 0.05;

    /**
     * <p>
     * Pose estimator standard deviation for encoder & gyro data
     * </p>
     * <b>Units:</b> Radians
     */
    public static final double measurementStdDevsHeading = Units.Radians.convertFrom(5, Units.Degrees);

    // This PID is implemented on each module, not the Drivetrain subsystem.
    public static final double driveP = 0.18;
    public static final double driveI = 0.0;
    public static final double driveD = 0;

    public static final double steerP = 100;
    public static final double steerI = 0.0;
    public static final double steerD = 0.14414076246334312;

    public static final double driveKs = 0;
    public static final double driveKa = 0;
    public static final double driveKv = (1 / driveSpeed);

    // This PID is implemented on the Drivetrain subsystem
    public static final double autoDriveP = 8;
    public static final double autoDriveI = 0;
    public static final double autoDriveD = 0;

    public static final double autoSteerP = 2.5;
    public static final double autoSteerI = 0.0;
    public static final double autoSteerD = 0.0;

    // Teleop Snapping to Rotation (Yaw)
    public static final double yawSnapP = 3;
    public static final double yawSnapI = 0;
    public static final double yawSnapD = 0;

  }

  public static final class prefVision {
    /**
     * <p>
     * Pose estimator standard deviation for vision data using Multi-tag
     * <p>
     * <b>Units:</b> Meters
     */
    public static final double multiTagStdDevsPosition = 0.7;

    /**
     * <p>
     * Pose estimator standard deviation for vision data using Multi-Tag
     * </p>
     * <b>Units:</b> Radians
     */
    public static final double multiTagStdDevsHeading = 9999999;

  }
}
