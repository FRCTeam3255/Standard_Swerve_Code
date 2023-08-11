// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.frcteam3255.utils.CTREModuleState;
import com.frcteam3255.utils.SN_Math;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.mapDrivetrain;
import frc.robot.RobotPreferences.prefDrivetrain;
import frc.robot.Constants.constDrivetrain;

// This subsystem represents 1 Swerve Module. The Drivetrain subsystem will reference 4 of these.
public class SwerveModule extends SubsystemBase {

  private TalonFX driveMotor;
  private TalonFX steerMotor;

  private TalonFXConfiguration driveConfiguration;
  private TalonFXConfiguration steerConfiguration;

  private CANCoder absoluteEncoder;
  private double absoluteEncoderOffset;

  public int moduleNumber;

  // Used when stopping the module (see: setModuleState();)
  private double lastAngleDegrees;

  public SwerveModule(int moduleNumber, int driveMotorID, int steerMotorID, int absoluteEncoderID,
      double absoluteEncoderOffset) {
    this.moduleNumber = moduleNumber;

    driveMotor = new TalonFX(driveMotorID, mapDrivetrain.CAN_BUS);
    steerMotor = new TalonFX(steerMotorID, mapDrivetrain.CAN_BUS);

    absoluteEncoder = new CANCoder(absoluteEncoderID, mapDrivetrain.CAN_BUS);
    this.absoluteEncoderOffset = absoluteEncoderOffset;

    driveConfiguration = new TalonFXConfiguration();
    steerConfiguration = new TalonFXConfiguration();

    lastAngleDegrees = 0;

    configure();
  }

  public void configure() {
    // -✨- Drive Motor Config -✨-
    driveMotor.configFactoryDefault();

    driveConfiguration.slot0.kF = prefDrivetrain.driveF.getValue();
    driveConfiguration.slot0.kP = prefDrivetrain.driveP.getValue();
    driveConfiguration.slot0.kI = prefDrivetrain.driveI.getValue();
    driveConfiguration.slot0.kD = prefDrivetrain.driveD.getValue();

    driveMotor.setNeutralMode(constDrivetrain.DRIVE_NEUTRAL_MODE);
    driveMotor.setInverted(constDrivetrain.DRIVE_MOTOR_INVERT);

    driveMotor.configAllSettings(driveConfiguration);

    // -✨- Steer Motor Config -✨-
    steerMotor.configFactoryDefault();

    steerConfiguration.slot0.kP = prefDrivetrain.steerP.getValue();
    steerConfiguration.slot0.kI = prefDrivetrain.steerI.getValue();
    steerConfiguration.slot0.kD = prefDrivetrain.steerD.getValue();

    steerMotor.setNeutralMode(constDrivetrain.STEER_NEUTRAL_MODE);
    steerMotor.setInverted(constDrivetrain.STEER_MOTOR_INVERT);

    steerMotor.configAllSettings(steerConfiguration);

    // -✨- Absolute Encoder Config -✨-
    absoluteEncoder.configFactoryDefault();

    lastAngleDegrees = getModuleState().angle.getDegrees();
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
        constDrivetrain.STEER_GEAR_RATIO);

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
        constDrivetrain.WHEEL_CIRCUMFERENCE,
        constDrivetrain.DRIVE_GEAR_RATIO);

    Rotation2d angle = Rotation2d.fromDegrees(
        SN_Math.falconToDegrees(
            steerMotor.getSelectedSensorPosition(),
            constDrivetrain.STEER_GEAR_RATIO));

    return new SwerveModuleState(velocity, angle);
  }

  /**
   * Get the current position of the module. This includes it's distance traveled
   * in meters and angle.
   * 
   * @return Module's SwerveModulePosition (distance, angle)
   */
  public SwerveModulePosition getModulePosition() {
    double distance = SN_Math.falconToMeters(
        driveMotor.getSelectedSensorPosition(),
        constDrivetrain.WHEEL_CIRCUMFERENCE,
        constDrivetrain.DRIVE_GEAR_RATIO);

    Rotation2d angle = Rotation2d.fromDegrees(
        SN_Math.falconToDegrees(
            steerMotor.getSelectedSensorPosition(),
            constDrivetrain.STEER_GEAR_RATIO));

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
    SwerveModuleState state = CTREModuleState.optimize(desiredState, getModuleState().angle);

    // -✨- Setting the Drive Motor -✨-

    if (isOpenLoop) {
      // Setting the motor to PercentOutput uses a percent of the motors max output.
      // So, the requested speed divided by it's max speed.
      double percentOutput = state.speedMetersPerSecond / constDrivetrain.MAX_MODULE_SPEED;
      driveMotor.set(ControlMode.PercentOutput, percentOutput);

    } else {
      // Convert velocity to Falcon encoder counts
      double velocity = SN_Math.MPSToFalcon(state.speedMetersPerSecond,
          constDrivetrain.WHEEL_CIRCUMFERENCE,
          constDrivetrain.DRIVE_GEAR_RATIO);

      driveMotor.set(ControlMode.Velocity, velocity);
    }

    // -✨- Setting the Steer Motor -✨-

    // Convert angle to Falcon encoder counts
    double angle = SN_Math.degreesToFalcon(
        state.angle.getDegrees(),
        constDrivetrain.STEER_GEAR_RATIO);

    // If the requested speed is lower than a relevant steering speed (ex.
    // jittering),
    // don't turn the motor. Set it to whatever it's previous angle was.
    if (Math.abs(state.speedMetersPerSecond) < (prefDrivetrain.minimumSteerSpeedPercent.getValue()
        * constDrivetrain.MAX_MODULE_SPEED)) {
      angle = lastAngleDegrees;
      return;
    }

    steerMotor.set(ControlMode.Position, angle);

    lastAngleDegrees = angle;
  }
}
