// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.frcteam3255.utils.CTREModuleState;
import com.frcteam3255.utils.SN_Math;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.SwerveConstants;

// This subsystem represents 1 Swerve Module. The Drivetrain subsystem will reference 4 of these.
public class SwerveModule extends SubsystemBase {

  // -✨- Module-Specific -✨-
  private TalonFX driveMotor;
  private TalonFX steerMotor;

  private CANCoder absoluteEncoder;
  private double absoluteEncoderOffset;

  public int moduleNumber;

  // -✨- Static Motor Config -✨-
  public static TalonFXConfiguration driveConfiguration;
  public static TalonFXConfiguration steerConfiguration;
  public static boolean isDriveInverted = false;
  public static NeutralMode driveNeutralMode = NeutralMode.Brake;
  public static boolean isSteerInverted = true;
  public static NeutralMode steerNeutralMode = NeutralMode.Coast;
  public static String CANBusName = "Swerve";
  public static double minimumSteerSpeedPercent = 0.01;

  // -✨- Static Physical Constants -✨-
  // These default to L2s, but should be overridden
  public static double driveGearRatio = SwerveConstants.MK4I_L2.driveGearRatio;
  public static double steerGearRatio = SwerveConstants.MK4I_L2.steerGearRatio;
  public static double wheelCircumference = SwerveConstants.MK4I_L2.wheelCircumference;
  public static double maxModuleSpeed = SwerveConstants.MK4I_L2.maxSpeed;

  // -✨- Sim -✨-
  private SwerveModuleState lastDesiredSwerveModuleState = new SwerveModuleState(0, new Rotation2d(0));
  private double desiredDrivePosition;
  private double timeFromLastUpdate;
  private double lastSimTime;
  private Timer simTimer;

  public SwerveModule(int moduleNumber, int driveMotorID, int steerMotorID, int absoluteEncoderID,
      double absoluteEncoderOffset) {
    if (Robot.isSimulation()) {
      simTimer = new Timer();
      simTimer.start();
      lastSimTime = simTimer.get();
      timeFromLastUpdate = 0;
    }

    this.moduleNumber = moduleNumber;

    driveMotor = new TalonFX(driveMotorID, CANBusName);
    steerMotor = new TalonFX(steerMotorID, CANBusName);

    absoluteEncoder = new CANCoder(absoluteEncoderID, CANBusName);
    this.absoluteEncoderOffset = absoluteEncoderOffset;

    driveConfiguration = new TalonFXConfiguration();
    steerConfiguration = new TalonFXConfiguration();
  }

  public void configure() {
    // -✨- Drive Motor Config -✨-
    driveMotor.configFactoryDefault();

    driveMotor.setNeutralMode(driveNeutralMode);
    driveMotor.setInverted(isDriveInverted);

    driveMotor.configAllSettings(driveConfiguration);

    // -✨- Steer Motor Config -✨-
    steerMotor.configFactoryDefault();

    steerMotor.setNeutralMode(steerNeutralMode);
    steerMotor.setInverted(isSteerInverted);

    steerMotor.configAllSettings(steerConfiguration);

    // -✨- Absolute Encoder Config -✨-
    absoluteEncoder.configFactoryDefault();
  }

  /**
   * Get the current raw position (no offset applied) of the module's absolute
   * encoder. This value will NOT match the physical angle of the wheel.
   * 
   * @return Position in degrees
   */
  public double getRawAbsoluteEncoder() {
    return absoluteEncoder.getAbsolutePosition();
  }

  /**
   * Get the current position, with the offset applied, of the module's absolute
   * encoder. This value should match the physical angle of the module's wheel.
   * 
   * @return Position in degrees, with the module's offset
   */
  public double getAbsoluteEncoder() {
    double degrees = getRawAbsoluteEncoder();

    // "This could make the value negative but it doesn't matter." - Ian 2023
    // Not 100% sure why but it's probably because it's a continuous circle
    degrees -= absoluteEncoderOffset;

    return degrees;
  }

  /**
   * Reset the steer motor encoder to the absolute encoder's value. The drive
   * motor is
   * not reset here because the absolute encoder does not record it's value.
   */
  public void resetSteerMotorToAbsolute() {
    double absoluteEncoderCount = SN_Math.degreesToFalcon(
        getAbsoluteEncoder(),
        steerGearRatio);

    steerMotor.setSelectedSensorPosition(absoluteEncoderCount);
  }

  /**
   * Reset the drive motor encoder to 0.
   */
  public void resetDriveMotorEncoder() {
    driveMotor.setSelectedSensorPosition(0);
  }

  /**
   * Get the current state of the module. This includes it's velocity and angle.
   * 
   * @return Module's SwerveModuleState (velocity, angle)
   */
  public SwerveModuleState getModuleState() {

    double velocity = SN_Math.falconToMPS(
        driveMotor.getSelectedSensorVelocity(),
        wheelCircumference,
        driveGearRatio);

    Rotation2d angle = Rotation2d.fromDegrees(
        SN_Math.falconToDegrees(
            steerMotor.getSelectedSensorPosition(),
            steerGearRatio));

    return new SwerveModuleState(velocity, angle);
  }

  /**
   * Get the current position of the module. This includes it's distance traveled
   * in meters and angle.
   * 
   * @return Module's SwerveModulePosition (distance, angle)
   */
  public SwerveModulePosition getModulePosition() {
    if (Robot.isSimulation()) {
      timeFromLastUpdate = simTimer.get() - lastSimTime;
      lastSimTime = simTimer.get();
      desiredDrivePosition += (lastDesiredSwerveModuleState.speedMetersPerSecond * timeFromLastUpdate);

      return new SwerveModulePosition(desiredDrivePosition, lastDesiredSwerveModuleState.angle);
    }

    double distance = SN_Math.falconToMeters(
        driveMotor.getSelectedSensorPosition(),
        wheelCircumference,
        driveGearRatio);

    Rotation2d angle = Rotation2d.fromDegrees(
        SN_Math.falconToDegrees(
            steerMotor.getSelectedSensorPosition(),
            steerGearRatio));

    return new SwerveModulePosition(distance, angle);
  }

  /**
   * Neutral the drive motor output.
   */
  public void neutralDriveOutput() {
    driveMotor.neutralOutput();
  }

  /**
   * Set the current state (velocity and position) of the module. Given values
   * are optimized so that the module can travel the least distance to achieve
   * the desired value.
   * 
   * @param desiredState Desired velocity and angle of the module
   * @param isOpenLoop   Is the module being set based on open loop or closed loop
   *                     control
   * 
   */
  public void setModuleState(SwerveModuleState desiredState, boolean isOpenLoop) {
    lastDesiredSwerveModuleState = desiredState;

    // Optimize explanation: https://youtu.be/0Xi9yb1IMyA?t=226
    SwerveModuleState state = CTREModuleState.optimize(desiredState, getModuleState().angle);
    // -✨- Setting the Drive Motor -✨-

    if (isOpenLoop) {
      // Setting the motor to PercentOutput uses a percent of the motors max output.
      // So, the requested speed divided by it's max speed.
      double percentOutput = state.speedMetersPerSecond / maxModuleSpeed;
      driveMotor.set(ControlMode.PercentOutput, percentOutput);

    } else {
      double velocity = SN_Math.MPSToFalcon(state.speedMetersPerSecond,
          wheelCircumference,
          driveGearRatio);

      driveMotor.set(ControlMode.Velocity, velocity);
    }

    // -✨- Setting the Steer Motor -✨-

    double angle = SN_Math.degreesToFalcon(
        state.angle.getDegrees(),
        steerGearRatio);

    // If the requested speed is lower than a relevant steering speed,
    // don't turn the motor. Set it to whatever it's previous angle was.
    if (Math.abs(state.speedMetersPerSecond) < (minimumSteerSpeedPercent
        * maxModuleSpeed)) {
      return;
    }
    steerMotor.set(ControlMode.Position, angle);
  }
}
