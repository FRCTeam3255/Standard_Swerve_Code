// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.Kilograms;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.frcteam3255.components.swerve.SN_SwerveConstants;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import frc.robot.Robot;

/**
 * The {@code ConstDrivetrain} class serves as a centralized repository for all
 * configuration constants related to the robot's drivetrain subsystem.
 * <p>
 * This class contains static final fields for hardware configuration (such as
 * absolute encoder offsets, gear ratios, wheel dimensions), control parameters
 * (such as speed multipliers, inversion flags), and physical measurements
 * (such as track width and wheelbase). These constants are used throughout the
 * robot code to ensure consistent and maintainable configuration of the
 * drivetrain.
 * <p>
 * Usage: Reference these constants statically wherever drivetrain configuration
 * values are required. This approach helps avoid magic numbers and makes it
 * easier to update configuration values in a single location.
 */
public class ConstDrivetrain {
  // TODO: Convert all applicable fields to MEASUREs

  // In Rotations: Obtain by aligning all of the wheels in the correct direction
  // and copy-pasting the Raw Absolute Encoder value

  // TODO: Swoffsets
  public static final double FRONT_LEFT_ABS_ENCODER_OFFSET = 0.417236;
  public static final double FRONT_RIGHT_ABS_ENCODER_OFFSET = -0.254395;
  public static final double BACK_LEFT_ABS_ENCODER_OFFSET = 0.258789;
  public static final double BACK_RIGHT_ABS_ENCODER_OFFSET = -0.290039;

  public static final double SLOW_MODE_MULTIPLIER = 0.5;

  public static final boolean INVERT_ROTATION = !Robot.isSimulation();

  public static final SN_SwerveConstants SWERVE_CONSTANTS = new SN_SwerveConstants(
      SN_SwerveConstants.MK4I.FALCON.L3.steerGearRatio,
      0.09779 * Math.PI,
      SN_SwerveConstants.MK4I.FALCON.L3.driveGearRatio,
      SN_SwerveConstants.MK4I.FALCON.L3.maxSpeedMeters);

  public static final double WHEEL_DIAMETER = SWERVE_CONSTANTS.wheelCircumference / Math.PI;
  public static final Distance WHEEL_RADIUS = Units.Meters.of(WHEEL_DIAMETER / 2);

  /**
   * <p>
   * Observed maximum translational speed while manually driving on the
   * Competition Robot.
   * </p>
   */
  public static final LinearVelocity REAL_DRIVE_SPEED = Units.FeetPerSecond.of(15.1);
  // Physically measured from center to center of the wheels
  // Distance between Left & Right Wheels for 25 by 25 frame
  public static final double TRACK_WIDTH_25 = Units.Meters.convertFrom(19.75, Units.Inches);
  // Distance between Front & Back Wheels for 25 by 25 frame
  public static final double WHEELBASE_25 = Units.Meters.convertFrom(19.75, Units.Inches);

  // Distance between Left & Right Wheels for 29 by 29 frame
  public static final double TRACK_WIDTH_29 = Units.Meters.convertFrom(23.75, Units.Inches);
  // Distance between Front & Back Wheels for 29 by 29 frame
  public static final double WHEELBASE_29 = Units.Meters.convertFrom(23.75, Units.Inches);

  // Distance between Left & Right Wheels
  public static final double TRACK_WIDTH = TRACK_WIDTH_29; // TODO: Replace with actual measurement
  // Distance between Front & Back Wheels
  public static final double WHEELBASE = WHEELBASE_29; // TODO: Replace with actual measurement

  // -- Pose Estimation --
  /**
   * <p>
   * Pose estimator standard deviation for encoder & gyro data
   * </p>
   * <b>Units:</b> Meters
   */
  public static final double MEASUREMENT_STD_DEVS_POS = 0.05;

  /**
   * <p>
   * Pose estimator standard deviation for encoder & gyro data
   * </p>
   * <b>Units:</b> Radians
   */
  public static final double MEASUREMENT_STD_DEV_HEADING = Units.Radians.convertFrom(5, Units.Degrees);

  // -- CONFIGS --
  public static TalonFXConfiguration DRIVE_CONFIG = new TalonFXConfiguration();
  public static TalonFXConfiguration STEER_CONFIG = new TalonFXConfiguration();
  public static CANcoderConfiguration CANCODER_CONFIG = new CANcoderConfiguration();

  // This config is kept separate as it's also used in the MODULE_CONFIG :p
  public static final Current DRIVE_CURRENT_LIMIT = Units.Amps.of(99999);

  public static final double MIN_STEER_PERCENT = 0.01;

  // Rotational speed (degrees per second) while manually driving
  public static final AngularVelocity TURN_SPEED = Units.DegreesPerSecond.of(360);

  // -- Motor Configurations --
  static {
    // This PID is implemented on each module, not the Drivetrain subsystem.
    // TODO: PID
    DRIVE_CONFIG.Slot0.kP = 0.18;
    DRIVE_CONFIG.Slot0.kI = 0.0;
    DRIVE_CONFIG.Slot0.kD = 0.0;
    DRIVE_CONFIG.Slot0.kS = 0.0;
    DRIVE_CONFIG.Slot0.kA = 0.0;
    DRIVE_CONFIG.Slot0.kV = (1 / REAL_DRIVE_SPEED.in(Units.MetersPerSecond));

    DRIVE_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    DRIVE_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    DRIVE_CONFIG.Feedback.SensorToMechanismRatio = SWERVE_CONSTANTS.driveGearRatio;
    DRIVE_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
    DRIVE_CONFIG.CurrentLimits.SupplyCurrentLimit = DRIVE_CURRENT_LIMIT.in(Units.Amps);

    STEER_CONFIG.Slot0.kP = 100;
    STEER_CONFIG.Slot0.kI = 0.0;
    STEER_CONFIG.Slot0.kD = 0.14414076246334312;

    STEER_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    STEER_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    STEER_CONFIG.Feedback.SensorToMechanismRatio = SWERVE_CONSTANTS.steerGearRatio;
    STEER_CONFIG.ClosedLoopGeneral.ContinuousWrap = true;

    CANCODER_CONFIG.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
  }

  public static class AUTO {
    // This PID is implemented on the Drivetrain subsystem
    // TODO: AUTO PID
    public static final PIDConstants AUTO_DRIVE_PID = new PIDConstants(9, 0.0, 0.0);

    public static final PIDConstants AUTO_STEER_PID = new PIDConstants(5.6, 0.0, 0.0);

    // Feet
    public static final double AUTO_MAX_SPEED = 8;
    // Feet per second
    public static final double AUTO_MAX_ACCEL = 6;

    public static final Mass MASS = Units.Kilograms.of(20);
    public static final double MOI = 8;
    public static final double WHEEL_COF = 1.0;
    public static final DCMotor DRIVE_MOTOR = DCMotor.getKrakenX60(1);
    public static final ModuleConfig MODULE_CONFIG = new ModuleConfig(WHEEL_RADIUS, REAL_DRIVE_SPEED, WHEEL_COF,
        DRIVE_MOTOR,
        DRIVE_CURRENT_LIMIT, 1);

    public static final Translation2d[] MODULE_OFFSETS = {
        new Translation2d(WHEELBASE / 2.0, TRACK_WIDTH / 2.0),
        new Translation2d(WHEELBASE / 2.0, -TRACK_WIDTH / 2.0),
        new Translation2d(-WHEELBASE / 2.0, TRACK_WIDTH / 2.0),
        new Translation2d(-WHEELBASE / 2.0, -TRACK_WIDTH / 2.0) };

    public static final RobotConfig ROBOT_CONFIG = new RobotConfig(MASS.in(Kilograms), MOI, MODULE_CONFIG,
        MODULE_OFFSETS);
  }

  public static class AUTO_ALIGN {
    public static final LinearVelocity MIN_DRIVER_OVERRIDE = ConstDrivetrain.REAL_DRIVE_SPEED.div(10);

    public static final PIDController POSE_TRANS_CONTROLLER = new PIDController(
        3,
        0,
        0);

    public static final PIDController PATH_TRANS_CONTROLLER = new PIDController(
        3,
        0,
        0);

    public static final Distance AT_POINT_TOLERANCE = Units.Inches.of(0.5);

    public static final ProfiledPIDController POSE_ROTATION_CONTROLLER = new ProfiledPIDController(
        2, 0, 0, new TrapezoidProfile.Constraints(TURN_SPEED.in(Units.DegreesPerSecond),
            Math.pow(TURN_SPEED.in(Units.DegreesPerSecond), 2)));

    public static final ProfiledPIDController PATH_ROTATION_CONTROLLER = new ProfiledPIDController(
        1, 0, 0, new TrapezoidProfile.Constraints(TURN_SPEED.in(Units.DegreesPerSecond),
            Math.pow(TURN_SPEED.in(Units.DegreesPerSecond), 2)));

    public static final Angle AT_ROTATION_TOLERANCE = Units.Degrees.of(1);

    static {
      POSE_TRANS_CONTROLLER.setTolerance(AT_POINT_TOLERANCE.in(Units.Meters));

      POSE_ROTATION_CONTROLLER.enableContinuousInput(0, 360);
      POSE_ROTATION_CONTROLLER.setTolerance(AT_ROTATION_TOLERANCE.in(Units.Degrees));
    }

    public static HolonomicDriveController POSE_AUTO_ALIGN_CONTROLLER = new HolonomicDriveController(
        POSE_TRANS_CONTROLLER,
        POSE_TRANS_CONTROLLER,
        POSE_ROTATION_CONTROLLER);

    public static HolonomicDriveController PATH_AUTO_ALIGN_CONTROLLER = new HolonomicDriveController(
        PATH_TRANS_CONTROLLER,
        PATH_TRANS_CONTROLLER,
        PATH_ROTATION_CONTROLLER);

  }
}