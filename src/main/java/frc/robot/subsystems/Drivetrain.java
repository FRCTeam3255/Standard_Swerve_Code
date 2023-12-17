// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SwerveConstants;
import frc.robot.Constants.constDrivetrain;
import frc.robot.RobotMap.mapDrivetrain;
import frc.robot.RobotPreferences.prefDrivetrain;
import frc.robot.RobotPreferences.prefVision;

public class Drivetrain extends SuperSwerve {
  private static TalonFXConfiguration driveConfiguration = new TalonFXConfiguration();
  private static TalonFXConfiguration steerConfiguration = new TalonFXConfiguration();

  private static SwerveModule[] modules = new SwerveModule[] {
      new SwerveModule(0, mapDrivetrain.FRONT_LEFT_DRIVE_CAN, mapDrivetrain.FRONT_LEFT_STEER_CAN,
          mapDrivetrain.FRONT_LEFT_ABSOLUTE_ENCODER_CAN, constDrivetrain.FRONT_LEFT_ABS_ENCODER_OFFSET),
      new SwerveModule(1, mapDrivetrain.FRONT_RIGHT_DRIVE_CAN, mapDrivetrain.FRONT_RIGHT_STEER_CAN,
          mapDrivetrain.FRONT_RIGHT_ABSOLUTE_ENCODER_CAN, constDrivetrain.FRONT_RIGHT_ABS_ENCODER_OFFSET),
      new SwerveModule(2, mapDrivetrain.BACK_LEFT_DRIVE_CAN, mapDrivetrain.BACK_LEFT_STEER_CAN,
          mapDrivetrain.BACK_LEFT_ABSOLUTE_ENCODER_CAN, constDrivetrain.BACK_LEFT_ABS_ENCODER_OFFSET),
      new SwerveModule(3, mapDrivetrain.BACK_RIGHT_DRIVE_CAN, mapDrivetrain.BACK_RIGHT_STEER_CAN,
          mapDrivetrain.BACK_RIGHT_ABSOLUTE_ENCODER_CAN, constDrivetrain.BACK_RIGHT_ABS_ENCODER_OFFSET),
  };

  public Drivetrain() {
    super(
        SwerveConstants.MK4I_L2,
        modules,
        constDrivetrain.WHEELBASE,
        constDrivetrain.TRACK_WIDTH,
        mapDrivetrain.CAN_BUS_NAME,
        mapDrivetrain.PIGEON_CAN,
        prefDrivetrain.minimumSteerSpeedPercent.getValue(),
        constDrivetrain.DRIVE_MOTOR_INVERT,
        constDrivetrain.STEER_MOTOR_INVERT,
        constDrivetrain.DRIVE_NEUTRAL_MODE,
        constDrivetrain.STEER_NEUTRAL_MODE,
        VecBuilder.fill(
            Units.feetToMeters(prefDrivetrain.measurementStdDevsFeet.getValue()),
            Units.feetToMeters(prefDrivetrain.measurementStdDevsFeet.getValue()),
            Units.degreesToRadians(prefDrivetrain.measurementStdDevsDegrees.getValue())),
        VecBuilder.fill(
            Units.feetToMeters(prefVision.measurementStdDevsFeet.getValue()),
            Units.feetToMeters(prefVision.measurementStdDevsFeet.getValue()),
            Units.degreesToRadians(prefVision.measurementStdDevsDegrees.getValue())),
        new PIDConstants(prefDrivetrain.autoDriveP.getValue(),
            prefDrivetrain.autoDriveI.getValue(),
            prefDrivetrain.autoDriveD.getValue()),
        new PIDConstants(prefDrivetrain.autoSteerP.getValue(),
            prefDrivetrain.autoSteerI.getValue(),
            prefDrivetrain.autoSteerD.getValue()),
        constDrivetrain.AUTO_FLIP_WITH_ALLIANCE_COLOR);

    exampleAuto = PathPlanner.loadPath("examplePath", new PathConstraints(
        Units.feetToMeters(prefDrivetrain.autoMaxSpeedFeet.getValue()),
        Units.feetToMeters(prefDrivetrain.autoMaxAccelFeet.getValue())));
  }

  @Override
  public void configure() {
    driveConfiguration.slot0.kF = prefDrivetrain.driveF.getValue();
    driveConfiguration.slot0.kP = prefDrivetrain.driveP.getValue();
    driveConfiguration.slot0.kI = prefDrivetrain.driveI.getValue();
    driveConfiguration.slot0.kD = prefDrivetrain.driveD.getValue();

    steerConfiguration.slot0.kP = prefDrivetrain.steerP.getValue();
    steerConfiguration.slot0.kI = prefDrivetrain.steerI.getValue();
    steerConfiguration.slot0.kD = prefDrivetrain.steerD.getValue();

    SwerveModule.driveConfiguration = driveConfiguration;
    SwerveModule.steerConfiguration = steerConfiguration;
    super.configure();
  }

  public void addEventToAutoMap(String key, Command command) {
    super.autoEventMap.put(key, command);
  }

  @Override
  public void periodic() {
    super.periodic();
  }
}
