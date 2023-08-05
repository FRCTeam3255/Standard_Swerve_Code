// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constDrivetrain;
import frc.robot.RobotMap.mapDrivetrain;
import frc.robot.RobotPreferences.prefDrivetrain;
import frc.robot.RobotPreferences.prefVision;

public class Drivetrain extends SubsystemBase {
  private SwerveModule[] modules;
  private SwerveDrivePoseEstimator swervePoseEstimator;
  private SwerveDriveKinematics swerveKinematics;
  private AHRS navX;
  private boolean isFieldRelative;

  public Drivetrain() {
    isFieldRelative = true;

    // to make this shorter you could do what Ian did but im lazy rn so im not gonna
    // do that.
    // TODO: DO THAT?
    modules = new SwerveModule[] {
        new SwerveModule(0, mapDrivetrain.FRONT_LEFT_DRIVE_CAN, mapDrivetrain.FRONT_LEFT_STEER_CAN,
            mapDrivetrain.FRONT_LEFT_ABSOLUTE_ENCODER_CAN, constDrivetrain.FRONT_LEFT_ABS_ENCODER_OFFSET),
        new SwerveModule(1, mapDrivetrain.FRONT_RIGHT_DRIVE_CAN, mapDrivetrain.FRONT_RIGHT_STEER_CAN,
            mapDrivetrain.FRONT_RIGHT_ABSOLUTE_ENCODER_CAN, constDrivetrain.FRONT_RIGHT_ABS_ENCODER_OFFSET),
        new SwerveModule(2, mapDrivetrain.BACK_LEFT_DRIVE_CAN, mapDrivetrain.BACK_LEFT_STEER_CAN,
            mapDrivetrain.BACK_LEFT_ABSOLUTE_ENCODER_CAN, constDrivetrain.BACK_LEFT_ABS_ENCODER_OFFSET),
        new SwerveModule(3, mapDrivetrain.BACK_RIGHT_DRIVE_CAN, mapDrivetrain.BACK_RIGHT_STEER_CAN,
            mapDrivetrain.BACK_RIGHT_ABSOLUTE_ENCODER_CAN, constDrivetrain.BACK_RIGHT_ABS_ENCODER_OFFSET),
    };
    swerveKinematics = constDrivetrain.SWERVE_KINEMATICS;

    navX = new AHRS();

    // Both the NavX and the absolute encoders need time to initialize before being
    // used
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        navX.reset();
        resetModulesToAbsolute();
      } catch (Exception e) {
      }
    }).start();

    swervePoseEstimator = new SwerveDrivePoseEstimator(
        swerveKinematics,
        navX.getRotation2d(),
        getModulePositions(),
        new Pose2d(),
        VecBuilder.fill(
            Units.feetToMeters(prefDrivetrain.measurementStdDevsFeet.getValue()),
            Units.feetToMeters(prefDrivetrain.measurementStdDevsFeet.getValue()),
            Units.degreesToRadians(prefDrivetrain.measurementStdDevsDegrees.getValue())),
        VecBuilder.fill(
            Units.feetToMeters(prefVision.measurementStdDevsFeet.getValue()),
            Units.feetToMeters(prefVision.measurementStdDevsFeet.getValue()),
            Units.degreesToRadians(prefVision.measurementStdDevsDegrees.getValue())));

  }

  public void resetModulesToAbsolute() {
    for (SwerveModule mod : modules) {
      mod.resetSteerMotorToAbsolute();
    }
  }

  /**
   * Get the rotation of the drivetrain using the NavX.
   * 
   * @return Rotation of drivetrain in radians
   */
  public Rotation2d getRotation() {
    return Rotation2d.fromRadians(MathUtil.angleModulus(navX.getRotation2d().getRadians()));
  }

  /**
   * Get the position (distance, angle) of each module.
   * 
   * @return An Array of Swerve module positions
   */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];

    for (SwerveModule mod : modules) {
      positions[mod.moduleNumber] = mod.getModulePosition();
    }

    return positions;
  }

  /**
   * Set the state of the modules
   * 
   * @param desiredModuleStates Desired states to set the modules to
   * @param isOpenLoop          Are the modules being set based on open loop or
   *                            closed loop
   *                            control
   * 
   */
  public void setModuleStates(SwerveModuleState[] desiredModuleStates, boolean isOpenLoop) {
    // Lowers the speeds so that they are actually achievable
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredModuleStates, constDrivetrain.MAX_MODULE_SPEED);

    for (SwerveModule mod : modules) {
      mod.setModuleState(desiredModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  /**
   * Drive the drivetrain
   * 
   * @param translation Desired translational velocity in meters per second
   * @param rotation    Desired rotational velocity in radians per second
   * @param isOpenLoop  Are the modules being set based on open loop or closed
   *                    loop
   *                    control
   * 
   */
  public void drive(Translation2d translation, double rotation, boolean isOpenLoop) {
    ChassisSpeeds chassisSpeeds;

    if (isFieldRelative) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          translation.getX(),
          translation.getY(),
          rotation,
          getRotation());
    } else {
      chassisSpeeds = new ChassisSpeeds(
          translation.getX(),
          translation.getY(),
          rotation);
    }

    SwerveModuleState[] desiredModuleStates = swerveKinematics.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(desiredModuleStates, isOpenLoop);
  }

  public void neutralDriveOutputs() {
    for (SwerveModule mod : modules) {
      mod.neutralDriveOutput();
    }
  }

  /**
   * Updates the pose estimator with the current robot uptime, the gyro yaw, and
   * each swerve module position.
   * <p>
   * This method MUST be called every loop (or else pose estimator breaks)
   */
  public void updatePoseEstimator() {
    swervePoseEstimator.updateWithTime(
        Timer.getFPGATimestamp(),
        navX.getRotation2d(),
        getModulePositions());
  }

  public void resetYaw() {
    navX.reset();
  }

  @Override
  public void periodic() {
    updatePoseEstimator();
    for (SwerveModule mod : modules) {
      SmartDashboard.putNumber("Module " + mod.moduleNumber + " Speed",
          Units.metersToFeet(mod.getModuleState().speedMetersPerSecond));
      SmartDashboard.putNumber("Module " + mod.moduleNumber + " Distance",
          Units.metersToFeet(mod.getModulePosition().distanceMeters));
      SmartDashboard.putNumber("Module " + mod.moduleNumber + " Angle",
          mod.getModuleState().angle.getDegrees());
      SmartDashboard.putNumber("Module " + mod.moduleNumber + " Absolute Encoder Angle (WITH OFFSET)",
          mod.getAbsoluteEncoder());
      SmartDashboard.putNumber("Module " + mod.moduleNumber + " Absolute Encoder Raw Value",
          mod.getRawAbsoluteEncoder());
    }
  }
}