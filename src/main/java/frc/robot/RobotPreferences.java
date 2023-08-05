package frc.robot;

import com.frcteam3255.preferences.SN_DoublePreference;

import edu.wpi.first.math.util.Units;

public class RobotPreferences {
  public static final class prefDrivetrain {
    public static final SN_DoublePreference driveF = new SN_DoublePreference("driveF", 0.045);
    public static final SN_DoublePreference driveP = new SN_DoublePreference("driveP", 0.1);
    public static final SN_DoublePreference driveI = new SN_DoublePreference("driveI", 0.0);
    public static final SN_DoublePreference driveD = new SN_DoublePreference("driveD", 1.0);

    public static final SN_DoublePreference steerP = new SN_DoublePreference("steerP", 0.3);
    public static final SN_DoublePreference steerI = new SN_DoublePreference("steerI", 0.0);
    public static final SN_DoublePreference steerD = new SN_DoublePreference("steerD", 6.0);

    public static final SN_DoublePreference minimumSteerSpeedPercent = new SN_DoublePreference("minimumSteerSpeed",
        0.01);

    // Translational speed in feet per second while driving using a controller
    // 16.3 FPS is maximum due to gearing
    public static final SN_DoublePreference driveSpeed = new SN_DoublePreference("driveSpeed", 16.3);

    // Rotational speed in degrees per second while driving using a controller
    // 943.751 DPS FPS is maximum due to gearing and robot size
    public static final SN_DoublePreference turnSpeed = new SN_DoublePreference("turnSpeed", 360);

    // Pose estimator standard deviations
    public static final SN_DoublePreference measurementStdDevsFeet = new SN_DoublePreference(
        "measurementStdDevsFeet", Units.metersToFeet(0.1));
    public static final SN_DoublePreference measurementStdDevsDegrees = new SN_DoublePreference(
        "measurementStdDevsDegrees", Units.metersToFeet(0.1));
  }

  public static final class prefVision {
    public static final SN_DoublePreference measurementStdDevsFeet = new SN_DoublePreference(
        "measurementStdDevsFeet", Units.metersToFeet(0.9));
    public static final SN_DoublePreference measurementStdDevsDegrees = new SN_DoublePreference(
        "measurementStdDevsDegrees", Units.metersToFeet(0.9));

  }
}
